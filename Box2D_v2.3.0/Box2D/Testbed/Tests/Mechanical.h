#ifndef MECHANICAL_H
#define MECHANICAL_H

class Mechanical : public Test
{
public:

	// Arms.
	// Shapes
	b2PolygonShape m_armA;
	b2PolygonShape m_armB;
	b2PolygonShape m_armC;
	b2PolygonShape m_armFixed;
	// to add shape to body
	b2FixtureDef fixDef_armA;
	b2FixtureDef fixDef_armB;
	b2FixtureDef fixDef_armC;
	b2FixtureDef fixDef_armFixed;
	// to store body
	b2Body* body_armA;
	b2Body* body_armB;
	b2Body* body_armC;
	b2Body* body_armFixed;
	// to store body def
	b2BodyDef bd_armA;
	b2BodyDef bd_armB;
	b2BodyDef bd_armC;
	b2BodyDef bd_armFixed;
	// to store fixture
	b2Fixture* m_armA_fix;
	b2Fixture* m_armB_fix;
	b2Fixture* m_armC_fix;
	b2Fixture* m_armFixed_fix;

	// Arm rev joints.
	// arm A to gear a, arm A to arm C top.
	b2RevoluteJoint* m_revArmA_1;
	b2RevoluteJoint* m_revArmA_2;
	b2RevoluteJointDef rjd_ArmA_1;
	b2RevoluteJointDef rjd_ArmA_2;
	// Arm B to gear B, Arm B to Arm C bottom.
	b2RevoluteJoint* m_revArmB_1;
	b2RevoluteJoint* m_revArmB_2;
	b2RevoluteJointDef rjd_ArmB_1;
	b2RevoluteJointDef rjd_ArmB_2;
	//Arm C to Arm Fixed.
	b2RevoluteJoint* m_revArmC_1;
	b2RevoluteJointDef rjd_ArmC_1;
	// Fixed arm to prismatic joint
	b2PrismaticJoint* m_prisArmFixed;
	b2PrismaticJointDef pjd_ArmFixed;

	// gear joints.
	b2Body* body;
	b2Body* body3;
	b2RevoluteJoint* m_revJointA;
	b2RevoluteJoint* m_revJointB;

	float angle;
	float radians;

	Mechanical()
	{

		angle = 90.0f;
		// Ground
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		// Gears and gear joint
		{
			// Gear A
			b2CircleShape shape;
			shape.m_radius = 5.0f;
			b2BodyDef bd;
			bd.type = b2_dynamicBody;

			b2CircleShape shape2;
			shape2.m_radius = 0.5f;
			b2BodyDef bd2;
			bd2.type = b2_staticBody;

			b2RevoluteJointDef rjd;
			
			bd.position.Set(3.0f, 19.0f);
			bd2.position.Set(3.0f, 19.0f);

			body = m_world->CreateBody(&bd);
			body->SetGravityScale(0.0f);
			body->CreateFixture(&shape, 5.0f);

			/*b2Body* body2 = m_world->CreateBody(&bd2);
			body2->SetGravityScale(0.0f);
			body2->CreateFixture(&shape2, 100.0f);*/

			rjd.Initialize(ground, body, body->GetPosition());
			rjd.motorSpeed = 1.0f * b2_pi;
			rjd.maxMotorTorque = 10000.0f;
			rjd.enableMotor = true;
			rjd.lowerAngle = 0;
			rjd.upperAngle = 0;
			rjd.enableLimit = false;
			rjd.collideConnected = false;

			m_revJointA = (b2RevoluteJoint*)m_world->CreateJoint(&rjd);

			// Gear B
			b2CircleShape shape3;
			shape3.m_radius = 4.0f;
			b2BodyDef bd3;
			bd3.type = b2_dynamicBody;

			/*b2CircleShape shape4;
			shape4.m_radius = 0.5f;
			b2BodyDef bd4;
			bd4.type = b2_staticBody;*/

			b2RevoluteJointDef rjd2;
			
			bd3.position.Set(4.0f, 10.0f);
			//bd4.position.Set(4.0f, 10.0f);

			body3 = m_world->CreateBody(&bd3);
			body3->SetGravityScale(0.0f);
			body3->CreateFixture(&shape3, 5.0f);

			/*b2Body* body4 = m_world->CreateBody(&bd4);
			body4->SetGravityScale(0.0f);
			body4->CreateFixture(&shape4, 100.0f);*/

			rjd2.Initialize(ground, body3, body3->GetPosition());
			rjd2.enableMotor = false;
			rjd2.collideConnected = false;

			m_revJointB = (b2RevoluteJoint*)m_world->CreateJoint(&rjd2);

			// Creating the gear joint
			b2GearJointDef jointDef;
			jointDef.bodyA = body;
			jointDef.bodyB = body3;
			jointDef.joint1 = m_revJointA;
			jointDef.joint2 = m_revJointB;
			jointDef.ratio = 1.0f;// * b2_pi;
			b2GearJoint* m_GearJoint = (b2GearJoint*)m_world->CreateJoint(&jointDef);
		//}

		// Mechanical arms
		//{
			float length = 5.0f;

			// Arm Fixed --------------------------------------
			bd_armFixed.type = b2_dynamicBody;
			bd_armFixed.position.Set(10.0f, 17.0f);
			body_armFixed = m_world->CreateBody(&bd_armFixed);
			body_armFixed->SetGravityScale(0.0f);

			m_armFixed.SetAsBox(length, 0.5f);

			fixDef_armFixed.shape = &m_armFixed;
			fixDef_armFixed.density = 1.0f;
			m_armFixed_fix = body_armFixed->CreateFixture(&fixDef_armFixed);

			pjd_ArmFixed.Initialize(body_armFixed, ground, body_armFixed->GetPosition(), b2Vec2(1, 0));
			pjd_ArmFixed.localAnchorA.Set( 4.0f, 0.0f);
			m_prisArmFixed = (b2PrismaticJoint*)m_world->CreateJoint(&pjd_ArmFixed);
			//-------------------------------------------------	

			// Arm C ------------------------------------------
			bd_armC.type = b2_dynamicBody;
			bd_armC.position.Set(10.0f, 15.0f);
			body_armC = m_world->CreateBody(&bd_armC);
			body_armC->SetGravityScale(0.0f);

			m_armC.SetAsBox(0.5f, 5.5f);
			fixDef_armC.shape = &m_armC;
			fixDef_armC.density = 1.0f;
			m_armC_fix = body_armC->CreateFixture(&fixDef_armC);

			rjd_ArmC_1.bodyA = body_armC;
			rjd_ArmC_1.bodyB = body_armFixed;
			rjd_ArmC_1.localAnchorA.Set( 0.0f, 0.0f);
			rjd_ArmC_1.localAnchorB.Set(-4.0f, 0.0f);
			m_revArmC_1 = (b2RevoluteJoint*)m_world->CreateJoint(&rjd_ArmC_1);

			// Arm A ------------------------------------------
			bd_armA.type = b2_dynamicBody;
			bd_armA.position.Set(5.0f, 20.0f);
			body_armA = m_world->CreateBody(&bd_armA);
			body_armA->SetGravityScale(0.0f);

			m_armA.SetAsBox(8.0f, 0.5f);
			fixDef_armA.shape = &m_armA;
			fixDef_armA.density = 1.0f;
			m_armA_fix = body_armA->CreateFixture(&fixDef_armA);

			// Joint from Arm A to gear A.
			rjd_ArmA_1.bodyA = body;
			rjd_ArmA_1.bodyB = body_armA;
			rjd_ArmA_1.localAnchorB.Set( -7.0f, 0.0f);
			rjd_ArmA_1.localAnchorA.Set(0.0f, 4.0f);
			m_revJointA = (b2RevoluteJoint*)m_world->CreateJoint(&rjd_ArmA_1);

			// Joint from Arm A to Arm C.
			rjd_ArmA_2.Initialize( body_armC, body_armA, body_armC->GetPosition());
			rjd_ArmA_2.localAnchorA.Set( 0.0f, 5.0f);
			rjd_ArmA_2.localAnchorB.Set(7.5f, 0.0f);
			m_revArmC_1 = (b2RevoluteJoint*)m_world->CreateJoint(&rjd_ArmA_2);

			// Arm B ------------------------------------------
			bd_armB.type = b2_dynamicBody;
			bd_armB.position.Set(5.0f, 15.0f);
			body_armB = m_world->CreateBody(&bd_armB);
			body_armB->SetGravityScale(0.0f);

			m_armB.SetAsBox(7.5f, 0.5f);
			fixDef_armB.shape = &m_armB;
			fixDef_armB.density = 1.0f;
			m_armB_fix = body_armB->CreateFixture(&fixDef_armB);

			// Joint from Arm B to wheel B
			rjd_ArmB_1.Initialize(body_armB, body3, body3->GetPosition());
			rjd_ArmB_1.bodyA = body_armB;
			rjd_ArmB_1.bodyB = body3;
			rjd_ArmB_1.localAnchorA.Set( -6.5f, 0.0f);
			rjd_ArmB_1.localAnchorB.Set(0.0f, 3.0f);
			rjd_ArmB_1.enableMotor = false;
			rjd_ArmB_1.collideConnected = false;
			m_revJointB = (b2RevoluteJoint*)m_world->CreateJoint(&rjd_ArmB_1);
			
			// Joint from Arm B to Arm C.
			rjd_ArmB_2.Initialize(body_armC, body_armB, body_armB->GetPosition());
			rjd_ArmB_2.localAnchorA.Set( 0.0f, -5.0f);
			rjd_ArmB_2.localAnchorB.Set(6.5f, 0.0f);
			rjd_ArmB_2.enableMotor = false;
			rjd_ArmB_2.collideConnected = false;
			m_revArmB_2 = (b2RevoluteJoint*)m_world->CreateJoint(&rjd_ArmB_2);
		}
	}

	void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
	{
		Test::PreSolve(contact, oldManifold);

		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
			case 'i':
				angle++;
				break;
			case 'k':
				angle--;
				break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		/*m_debugDraw.DrawString(5, m_textLine, "Radians = %4.2f" , radians);
		m_textLine += DRAW_STRING_NEW_LINE;
		m_debugDraw.DrawString(5, m_textLine, "Degrees =%4.2f ", angle);*/
	}

	static Test* Create()
	{
		return new Mechanical;
	}
	
};

#endif