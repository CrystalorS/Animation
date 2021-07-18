#include "Creature.h"
//#include <windows.h>
#define BILLION 1E9;
#include <iostream>
using namespace std;

Creature::Creature (b2World* world) : m_world (world), m_hasFallen(false), m_isTracking(false) { // Constructeur

		// Creation des corps rigides
		// ==========================

		// Proprietes communes
		b2BodyDef bodyDef;
		bodyDef.fixedRotation = false;
		bodyDef.allowSleep = false;
		bodyDef.awake = true;
		bodyDef.type = b2_dynamicBody;
		bodyDef.linearDamping = 0.01f;
		bodyDef.angularDamping = 0.01f;
        b2PolygonShape shape;
        b2CircleShape shape2;
		b2FixtureDef fixture;

		fixture.shape = &shape2;

		//TETE
		bodyDef.position.Set(0.0f,5.0f);
		m_bodies[TETE] = m_world->CreateBody(&bodyDef);
		shape2.m_radius = 0.4f;
        fixture.density = 7.5f;
        fixture.friction = 0.92;
		m_bodies[TETE]->CreateFixture(&fixture);

        fixture.shape = &shape;


		//BRAS_G
		bodyDef.position.Set(-0.8,3.8f);
		bodyDef.angle = -0.3f;
		m_bodies[BRAS_G] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.2f, 0.5f);
        fixture.density = 7.5f;
        fixture.friction = 0.92;
		m_bodies[BRAS_G]->CreateFixture(&fixture);

		//BRAS_D
		bodyDef.position.Set(0.8f,3.8f);
		bodyDef.angle = 0.3f;
		m_bodies[BRAS_D] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.2f, 0.5f);
        fixture.density = 7.5f;
        fixture.friction = 0.92;
		m_bodies[BRAS_D]->CreateFixture(&fixture);

        // TRONC
		bodyDef.position.Set(0.0f,3.8f);
        bodyDef.angle = 0.0f;
        m_bodies[TRONC] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.6f, 0.6f);
        fixture.density = 2.52f;
        m_bodies[TRONC]->CreateFixture(&fixture);

        // CUISSES
        bodyDef.position.Set(-0.8f,2.8f);
        m_bodies[CUISSE_G] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.2f, 0.4f);
        fixture.density = 2.52f;
        m_bodies[CUISSE_G]->CreateFixture(&fixture);

        bodyDef.position.Set(0.8f,2.8f);
        m_bodies[CUISSE_D] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.2f, 0.4f);
        fixture.density = 2.52f;
        m_bodies[CUISSE_D]->CreateFixture(&fixture);

        // JAMBES
        bodyDef.position.Set(-1.0f,2.0f);
        m_bodies[JAMBE_G] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.2f, 0.6f);
        fixture.density = 5.0f;
        m_bodies[JAMBE_G]->CreateFixture(&fixture);

        bodyDef.position.Set(1.0f,2.0f);
        m_bodies[JAMBE_D] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.2f, 0.6f);
        fixture.density = 5.0f;
        m_bodies[JAMBE_D]->CreateFixture(&fixture);

		// PIEDS
        bodyDef.position.Set(-1.0f,1.22f);
        m_bodies[PIED_G] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.4f, 0.1f);
        fixture.density = 7.5f;
        fixture.friction = 0.92;
        m_bodies[PIED_G]->CreateFixture(&fixture);

        bodyDef.position.Set(1.0f,1.22f);
        m_bodies[PIED_D] = m_world->CreateBody(&bodyDef);
        shape.SetAsBox(0.4f, 0.1f);
        fixture.density = 7.5f;
        fixture.friction = 0.92;
        m_bodies[PIED_D]->CreateFixture(&fixture);

        m_baseDistPieds = 2.0f;



		// Creation des articulations
		// ==========================

		// Proprietes communes
		b2RevoluteJointDef jointDef;
		jointDef.lowerAngle = -0.5f * b2_pi;
		jointDef.upperAngle = 0.5f * b2_pi;
		jointDef.enableLimit = true;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 100.0f;

		// COU
		jointDef.Initialize(m_bodies[TETE],m_bodies[TRONC],m_bodies[TETE]->GetWorldCenter()+b2Vec2(0.0f,0.2f));
		m_joints[COU] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);

		// EPAULE
		jointDef.Initialize(m_bodies[BRAS_G],m_bodies[TRONC],m_bodies[BRAS_G]->GetWorldCenter()+b2Vec2(0.0f,0.3f));
		m_joints[EPAULE_G] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
        // EPAULE2
		jointDef.Initialize(m_bodies[BRAS_D],m_bodies[TRONC],m_bodies[BRAS_D]->GetWorldCenter()+b2Vec2(0.0f,0.3f));
		m_joints[EPAULE_D] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);

        // HANCHE
		jointDef.Initialize(m_bodies[CUISSE_G],m_bodies[TRONC],m_bodies[CUISSE_G]->GetWorldCenter()+b2Vec2(0.4f,0.3f));
		m_joints[HANCHE_G] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
         // HANCHE
		jointDef.Initialize(m_bodies[CUISSE_D],m_bodies[TRONC],m_bodies[CUISSE_D]->GetWorldCenter()+b2Vec2(-0.4f,0.3f));
		m_joints[HANCHE_D] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);

        // GENOU
		jointDef.Initialize(m_bodies[JAMBE_G],m_bodies[CUISSE_G],m_bodies[JAMBE_G]->GetWorldCenter()+b2Vec2(0.0f,0.4f));
		m_joints[GENOU_G] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
         // GENOU
		jointDef.Initialize(m_bodies[JAMBE_D],m_bodies[CUISSE_D],m_bodies[JAMBE_D]->GetWorldCenter()+b2Vec2(0.0f,0.4f));
		m_joints[GENOU_D] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);


        // CHEVILLE
		jointDef.Initialize(m_bodies[PIED_G],m_bodies[JAMBE_G],m_bodies[PIED_G]->GetWorldCenter()+b2Vec2(0.0f,0.2f));
		m_joints[CHEVILLE_G] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);
        // CHEVILLE
		jointDef.Initialize(m_bodies[PIED_D],m_bodies[JAMBE_D],m_bodies[PIED_D]->GetWorldCenter()+b2Vec2(0.0f,0.2f));
		m_joints[CHEVILLE_D] = (b2RevoluteJoint*) m_world->CreateJoint(&jointDef);


		// Controleur
		// ==========
		m_PDControllers[COU] = new PDController(3.0,1.0);
		m_PDControllers[EPAULE_G] = new PDController(3.0,1.0);
		m_PDControllers[EPAULE_D] = new PDController(3.0,1.0);
		m_PDControllers[HANCHE_G] = new PDController(5.0, 1.0);
		m_PDControllers[HANCHE_D] = new PDController(5.0,1.0);
		m_PDControllers[GENOU_G] = new PDController(8.0, 4.0);
		m_PDControllers[GENOU_D] = new PDController(8.0, 4.0);
		m_PDControllers[CHEVILLE_G] = new PDController(4.0, 2.0);
		m_PDControllers[CHEVILLE_D] = new PDController(4.0,2.0);

		// Creation du mouvement de reference a suivre
		m_motion = new Motion("motion.txt");

}

Creature::~Creature() { // Destructor
		for (int i = 0; i < NB_ARTICULATIONS; ++i) {
			delete m_PDControllers[i]; m_PDControllers[i] = NULL;
		}
		delete m_motion;
}

void Creature::update(bool balanceControl, int walk, bool& jump, bool motionTracking) {

    // Remise a zero des moments articulaires
    for (int j = 0; j < NB_ARTICULATIONS; ++j) m_motorTarget[j] = 0.0;
    // Calcule et met a jour les moments articulaires necessaires a equilibrer la creature
    if (balanceControl) balanceController();
    // Calcule et met a jour les moments articulaires pour suivre le mouvement de reference
    if (motionTracking) motionTracker();
    else m_isTracking = false;

    if(walk != 0) walkController(walk);
    if(jump) jumpController(jump);
    // Applique les moments articulaires en tant que "motor speed"
     for (int j = 0; j < NB_ARTICULATIONS; ++j) m_joints[j]->SetMotorSpeed(m_motorTarget[j]);
}

void Creature::balanceController() {

	// BALANCE CONTROLLER
	// ==================

	// Calcul du CdM dans le repere du monde
	m_positionCOM = computeCenterOfMass();

	// Mettre a jour la pose seulement si la creature est debout
	if (m_hasFallen) return;

	// Etape 1.1: Decrire le CPS dans le repere du monde
	// definir la position du CPS localement a un corps rigide bien choisi
	// utiliser b2Body::GetTransform pour optenir la transformation decrivant position+orientation du corps rigide dans le monde
	// utiliser la transformation pour convertir la position locale du CPS en sa position globale

	b2Vec2 cpsG = b2Mul(m_bodies[PIED_G]->GetTransform(),b2Vec2(0.0f,-0.2f));
	b2Vec2 cpsD = b2Mul(m_bodies[PIED_D]->GetTransform(), b2Vec2(0.0f, -0.2f));

	//Il y a deux pieds donc on fait la moyenne
	b2Vec2 cpsMonde ((cpsG.x + cpsD.x) / 2.0f, (cpsG.y + cpsD.y) / 2.0f);

	// Etape 1.2: Calculer la force d'equilibre, en 3D, a appliquer a partir de la distance entre les CPS et CdM projetes au sol
	b2Vec3 forceToApply (cpsMonde.x - m_positionCOM.x,0.0f,0.0f);

	if (m_positionCOM.x <= m_baseDistPieds) return;

	// Etape 2.1: decrire la position, en 3D, de la cheville dans le repere du pied
	b2Vec3 positionOfAxisLocal(0.0f,0.2f,0.0f);
	// Etape 2.2: decrire l'axe de rotation, en 3D, de la cheville dans le repere du pied : z
    b2Vec3 axisOfRotationLocal (0.0f,0.0f,0.0f);
    // Etape 2.3: appeller jacobianTranspose afin d'estimer le moment necessaire a la cheville pour simuler l'effet de la force
    float jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),PIED_G,forceToApply);
    // Etape 2.4: mettre a l'echelle l'erreur en moment articulaire
    jointTorque *= 1.0f;
    // Etape 2.5: ajouter le resultat dans le tableau m_motorTarget
    m_motorTarget[CHEVILLE_G] += jointTorque;

	positionOfAxisLocal = b2Vec3(0.0f,0.2f,0.0f);
	axisOfRotationLocal = b2Vec3(0.0f,0.0f,1.f);
	jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),PIED_D,forceToApply);
	jointTorque *= 1.0f;
    m_motorTarget[CHEVILLE_D] += jointTorque;

	// GENOU GACUHE
	positionOfAxisLocal = b2Vec3(0.0f,0.4f,0.0f);
	axisOfRotationLocal = b2Vec3(0.0f,0.0f,1.0f);
	jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),JAMBE_G,forceToApply);
	jointTorque *= 1.0f;
    m_motorTarget[GENOU_G] += jointTorque;

	// GENOU DROIT
	positionOfAxisLocal = b2Vec3(0.0f,0.4f,0.0f);
	axisOfRotationLocal = b2Vec3(0.0f,0.0f,1.0f);
	jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),JAMBE_D,forceToApply);
	jointTorque *= 1.0f;
    m_motorTarget[GENOU_D] += jointTorque;

	// HANCHE GAUCHE
	positionOfAxisLocal = b2Vec3(-0.5f,0.3f,0.0f);
	axisOfRotationLocal = b2Vec3(0.0f,0.0f,1.0f);
	jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),CUISSE_G,forceToApply);
	jointTorque *= 1.0f;
    m_motorTarget[HANCHE_G] += jointTorque;

	// HANCHE DROITE
	positionOfAxisLocal = b2Vec3(0.5f,0.3f,0.0f);
	axisOfRotationLocal = b2Vec3(0.0f,0.0f,1.f);
	jointTorque = jacobianTranspose(positionOfAxisLocal,axisOfRotationLocal,b2Vec3(m_positionCOM.x,m_positionCOM.y,0.0f),CUISSE_D,forceToApply);
	jointTorque *= 1.0f;
    m_motorTarget[HANCHE_D] += jointTorque;

}

void Creature::articulation(int idArticulation, float angleTarget, float torqueMult){
  if(idArticulation >= NB_ARTICULATIONS) return;
  float anglec = m_joints[idArticulation]->GetJointAngle();
  m_PDControllers[idArticulation]->setTarget((double)angleTarget);
  double torque = m_PDControllers[idArticulation]->compute((double)anglec);
  torque *= torqueMult;
  m_motorTarget[idArticulation] += torque;

}

void Creature::walkController(int walk) {

	// walk CONTROLLER
	// ==================

	if (hasFallen()) return;

	if (walk != this->walk) walkStep = 0;
	this->walk = walk;

	float angleCG, angleCD, angleGG, angleGD, angleHG, angleHD;

	float currAngleHG = m_joints[HANCHE_G]->GetJointAngle();
	float currAngleHD = m_joints[HANCHE_D]->GetJointAngle();

	// Init pose
	if (walkStep == 0)
	{
		angleHG = -15.0f * (b2_pi / 180.0f);
		angleHD = 15.0f * (b2_pi / 180.0f);
		angleGG = -5.0f * (b2_pi / 180.0f);
		angleGD = 5.0f * (b2_pi / 180.0f);
		angleCG = 20.0f * (b2_pi / 180.0f);
		angleCD = -20.0f * (b2_pi / 180.0f);

		//Pour appliquer les moments
		articulation(HANCHE_G, angleHG, 5.0f);
		articulation(HANCHE_D, angleHD, 5.0f);

		articulation(GENOU_G, angleGG, 5.0f);
		articulation(GENOU_D, angleGD, 5.0f);

		articulation(CHEVILLE_G, angleCG, 5.0f);
		articulation(CHEVILLE_D, angleCD, 5.0f);


		float distAngle = abs(currAngleHD - angleHD);
		if (distAngle < 0.01f)
		{
			walkStep++;
		}
	}

	// Move leg
	if (walkStep == 1)
	{
		angleHG = 0.0f * (b2_pi / 180.0f);
		angleHD = 0.0f * (b2_pi / 180.0f);
		(walk < 0) ? angleGD = -30.0f * (b2_pi / 180.0f) : angleGD = 0.0f * (b2_pi / 180.0f);
		(walk > 0) ? angleGG = 30.0f * (b2_pi / 180.0f) : angleGG = 0.0f * (b2_pi / 180.0f);
		angleCG = 0.0f * (b2_pi / 180.0f);
		angleCD = 0.0f * (b2_pi / 180.0f);

		articulation(HANCHE_G, angleHG, 1.0f);
		articulation(HANCHE_D, angleHD, 1.0f);

		(walk < 0) ?
			articulation(GENOU_G, angleGG, 1.0f) :
			articulation(GENOU_G, angleGG, 10.0f);

		(walk > 0) ?
			articulation(GENOU_D, angleGD, 1.0f) :
			articulation(GENOU_D, angleGD, 10.0f);

		articulation(CHEVILLE_G, angleCG, 1.0f);
		articulation(CHEVILLE_D, angleCD, 1.0f);

		float distAngle;
		if (walk < 0) distAngle = abs(currAngleHG - angleHG);
		else if (walk > 0) distAngle = abs(currAngleHD - angleHD);
		if (distAngle < 0.01f)
		{
			walkStep++;
		}
	}

	// Move other leg to stabilize
	if (walkStep == 2)
	{
		angleHG = 0.0f * (b2_pi / 180.0f);
		angleHD = 0.0f * (b2_pi / 180.0f);
		(walk < 0) ? angleGG = -30.0f * (b2_pi / 180.0f) : angleGG = 0.0f * (b2_pi / 180.0f);
		(walk > 0) ? angleGD = 30.0f * (b2_pi / 180.0f) : angleGD = 0.0f * (b2_pi / 180.0f);
		angleCG = 0.0f * (b2_pi / 180.0f);
		angleCD = 0.0f * (b2_pi / 180.0f);

		articulation(HANCHE_G, angleHG, 1.0f);
		articulation(HANCHE_D, angleHD, 1.0f);

		articulation(GENOU_G, angleGG, 10.0f);
		articulation(GENOU_D, angleGD, 10.0f);

		articulation(CHEVILLE_G, angleCG, 1.0f);
		articulation(CHEVILLE_D, angleCD, 1.0f);

		float distAngle;
		if (walk < 0) distAngle = abs(currAngleHG - angleHG);
		else if (walk > 0) distAngle = abs(currAngleHD - angleHD);
		if (distAngle < 0.01f)
		{
			walkStep = 0;
		}
	}
}



void Creature::jumpController(bool& jumpControl)
{
    if (hasFallen()) return;

	b2Vec2 jumpTarget (m_bodies[TRONC]->GetPosition().x, 4.4f);
	b2Vec2 appuiTarget (m_bodies[TRONC]->GetPosition().x, 3.2f);
	b2Vec2 current (m_bodies[TRONC]->GetPosition().x, m_bodies[TRONC]->GetPosition().y);

	//g_debugDraw.DrawCircle(b2Vec2(jumpTarget.x, jumpTarget.y), 0.1f, b2Color(0.0f, 255.0f, 0.0f));
    //g_debugDraw.DrawCircle(b2Vec2(appuiTarget.x, appuiTarget.y), 0.1f, b2Color(255.0f, 0.0f, 0.0f));
    //g_debugDraw.DrawCircle(b2Vec2(current.x, current.y), 0.1f, b2Color(0.0f, 0.0f, 255.0f));

	float angleC, angleG, angleH;

	float length;
	float minDist;

	// PHASE D'APPUI
	if (jumpStep == 0)
	{
		length = b2Distance(appuiTarget, current);
		minDist = 0.125f;
		if (length > minDist)
		{
			angleH = -60.0f * (b2_pi / 180.0f);
			angleG = 80.0f * (b2_pi / 180.0f);
			angleC = 50.0f * (b2_pi / 180.0f);

			articulation(HANCHE_G, angleH, 3.0f);
			articulation(HANCHE_D, -angleH, 3.0f);

			articulation(GENOU_G, -angleG, 5.0f);
			articulation(GENOU_D, angleG, 5.0f);

			articulation(CHEVILLE_G, angleC, 8.0f);
			articulation(CHEVILLE_D, -angleC, 8.0f);
		}
		else
		{
			jumpStep++;
		}
	}

	// PHASE DE SAUT
	if (jumpStep == 1)
	{
		length = b2Distance(jumpTarget, current);
		minDist = 0.125f;

		if (length > minDist)
		{
			angleH = -80.0f * (b2_pi / 180.0f);
			angleG = -60.0f * (b2_pi / 180.0f);
			angleC = -15.0f * (b2_pi / 180.0f);

			articulation(HANCHE_G, angleH, 50.0f);
			articulation(HANCHE_D, -angleH, 50.0f);

			articulation(GENOU_G, -angleG, 30.0f);
			articulation(GENOU_D, angleG, 30.0f);

			articulation(CHEVILLE_G, angleC, 20.0f);
			articulation(CHEVILLE_D, -angleC, 20.0f);
		}
		else
		{
			jumpStep++;
		}
	}

	// PHASE EN L'AIR
	if (jumpStep == 2)
	{
        //std::cout<<"Je retombe sur mes pieds"<<std::endl;
		angleH = 0.0f * (b2_pi / 180.0f);
		angleG = 0.0f * (b2_pi / 180.0f);
		angleC = 0.0f * (b2_pi / 180.0f);

		articulation(HANCHE_G, angleH, 2.0f);
		articulation(HANCHE_D, -angleH, 2.0f);

		articulation(GENOU_G, -angleG, 2.0f);
		articulation(GENOU_D, angleG, 2.0f);

		articulation(CHEVILLE_G, angleC, 2.0f);
		articulation(CHEVILLE_D, -angleC, 2.0f);

		// Distance entre pos acutelle et position initiale des pieds
		float distPiedG = abs(m_bodies[PIED_G]->GetPosition().y - 1.22f);
		float distPiedD = abs(m_bodies[PIED_D]->GetPosition().y - 1.22f);
		if (distPiedG < 0.1f && distPiedD < 0.1f)
		{
			// Retour en phase initiale
			jumpStep = 0;
			jumpControl = false;
		}
	}
}

timespec Creature::GetTickCount(timespec time){
  clock_gettime(CLOCK_REALTIME, &time);
  return time;
}

void Creature::motionTracker() {
    struct timespec requestStart,requestEnd;
    // Activation du suivi
    if (!m_isTracking) {m_isTracking=true;m_motion->setStartTime(GetTickCount(requestStart));}
    // Calculer le temps depuis le debut du suivi (en ms)
    requestEnd = GetTickCount(requestEnd);
    double elapsedTime = ( requestEnd.tv_sec - requestStart.tv_sec ) + ( requestEnd.tv_nsec - requestStart.tv_nsec ) / BILLION;
    // Calculer l'indice dans le mouvement
    unsigned int frameIndex = (unsigned int)(elapsedTime / (100 * m_motion->getFrequency())) % m_motion->getNbFrames();
    // Recuperer les donnees correspondant a l'indice
    std::vector<float> frameData = m_motion->getMotionDataAtFrame(frameIndex);
    // Calculer le moment pour chaque articulation
    for (int j = 0; j < NB_ARTICULATIONS; ++j) {
        // Lire l'angle pour l'articulation j
        float targetAngle = frameData[j];
        // Affecter l'angle cible pour le regulateur PD
        m_PDControllers[j]->setTarget(targetAngle);
        // Lire l'angle actuel
        float32 currentAngle = m_joints[j]->GetJointAngle();
        // Calculer le moment (appel a PDController::compute)
        float trackingMotor = m_PDControllers[j]->compute(currentAngle);
        // Ajouter le resultat dans le tableau m_motorTarget
        m_motorTarget[j] += trackingMotor;
    }

}

bool Creature::hasFallen() {
	if (m_hasFallen) return m_hasFallen; // vrai si deja a terre
	if (m_bodies[TRONC]->GetWorldCenter().y < 1.22 ) {
            // detection que la creature est tombee (le CdM du tronc est au niveau de la plateforme)
            for (int j = 0; j < NB_ARTICULATIONS; ++j) {
                   m_joints[j]->SetMotorSpeed(0.0f);
                   m_joints[j]->EnableMotor(false);
            }
            m_hasFallen = true;
	}
	return m_hasFallen;
}

b2Vec2 Creature::computeCenterOfMass() {
    float32 total_mass = 0.0f;
    b2Vec2 sum_COMs(0.0f,0.0f);
    for (int i = 0; i < NB_CORPS; ++i) {
        float32 massBody = m_bodies[i]->GetMass();
        b2Vec2 comBody = m_bodies[i]->GetWorldCenter();
        sum_COMs += massBody * comBody;
        total_mass += massBody;
    }
    return (1.0f / total_mass) * sum_COMs;
}

float Creature::jacobianTranspose(b2Vec3 positionOfAxisLocal, b2Vec3 axisOfRotationLocal, b2Vec3 positionApplyForce, int ID_RIGIDBODY, b2Vec3 forceToApply) {
    // Convertir la position locale de l'articulation en position dans le monde
    b2Vec2 positionOfAxisLocal2D (positionOfAxisLocal.x,positionOfAxisLocal.y);
    b2Vec2 positionOfAxisInWorld2D = b2Mul(m_bodies[ID_RIGIDBODY]->GetTransform(),positionOfAxisLocal2D);
    b2Vec3 positionOfAxisInWorld (positionOfAxisInWorld2D.x,positionOfAxisInWorld2D.y,0.0f);
	// Convertir l'orientation de l'axe de rotation locale dans le monde (rien a faire en 2D)
	b2Vec3 axisOfRotationWorld = axisOfRotationLocal;
	// Calculer le moment articulaire le long de l'axe de rotation (formule de la transposee de Jacobienne)
	float jointTorque = b2Dot(b2Cross(axisOfRotationWorld,(positionApplyForce - positionOfAxisInWorld)),forceToApply);
	// Retourner le moment articulaire
	return jointTorque;
}
