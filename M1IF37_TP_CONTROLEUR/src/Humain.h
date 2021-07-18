#ifndef HUMAIN_H
#define HUMAIN_H



#include "Box2D/Box2D.h"
#include "PDController.h"
#include "Motion.h"

class Humain {

public:
	Humain (b2World* world);               // Constructeur
	virtual	~Humain();                     // Destructeur

	void update(bool balanceControl, bool motionTracking);  // Mise a jour de la creature (controleur)
	bool hasFallen();					                    // Vrai si la creature est tombee
    timespec GetTickCount(timespec time);
	b2Vec2 getCOM() {return m_positionCOM;}	            // Retourne la position du CdM

protected:

	enum {PIED,JAMBE,TRONC,NB_CORPS};           // Segments de l'Humain
	enum {CHEVILLE,HANCHE,NB_ARTICULATIONS};    // Articulations de la creature

	b2World             *	m_world;		    		        // Le monde physique
	b2Body			    *	m_bodies[NB_CORPS];                 // Le tableau de corps rigides
	b2RevoluteJoint     *	m_joints[NB_ARTICULATIONS];		    // Le tableau d'articulations
	float                   m_motorTarget[NB_ARTICULATIONS];    // Le tableau de moments articulaires a appliquer
	PDController		*	m_PDControllers[NB_ARTICULATIONS];	// Le tableau de regulateurs PD (un par articulation)

	bool	    m_hasFallen;            // Vrai si la creature est tombee
	bool        m_isTracking;           // Vrai si le suivi est active
	Motion  *   m_motion;               // Le mouvement a suivre
	b2Vec2      m_positionCOM;          // La position du CdM

	b2Vec2 computeCenterOfMass();                               // Calcule la position du CdM de la creature dans le repere du monde
	void balanceController();                                   // Calcule et met a jour les moments articulaires pour equilibrer la creature
	void motionTracker();                                       // Calcule et met a jour les moments articulaires pour suivre le mouvement de reference
	float jacobianTranspose(b2Vec3,b2Vec3,b2Vec3,int,b2Vec3);   // Calcule les moments necessaires a simuler les effets de la force (en 3D)

};

#endif// HUMAIN_H
