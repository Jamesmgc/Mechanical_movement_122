#ifndef MECHANICAL_H
#define MECHANICAL_H

class Mechanical : public Test
{
public:
	Mechanical()
	{
		// Collision filtering for gear teeth and arms.
		// Teeth collide with teeth but not arms.
		const uint16 k_defaultCategory = 0x0001;
		const uint16 k_toothCategory = 0x0002;
		const uint16 k_armCategory = 0x0004;
		// What the category collides with, 0xFFFF = everything.
		// Only collides with other arms.
		const uint16 k_armMask = k_armCategory;
		//collides with everything apart from arms.
		const uint16 k_toothMask = 0xFFFF;

		// Number of teeth on each gear.
		numOfTeeth_A = 10;
		numOfTeeth_B = 8;

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
			shape.m_radius = 6.0f;
			b2BodyDef bd;
			bd.type = b2_dynamicBody;

			b2CircleShape shape2;
			shape2.m_radius = 0.5f;//5

			b2RevoluteJointDef rjd;
			
			bd.position.Set(0.0f, 25.0f);//3.0f, 19.0f

			body = m_world->CreateBody(&bd);
			body->SetGravityScale(0.0f);
			body->CreateFixture(&shape, 5.0f);

			// Placing each tooth on the gear.
			// Shape of the tooth
			b2PolygonShape tooth_shape;
			tooth_shape.SetAsBox(1.2f, 1.0f);

			// Centre of rotation, being the centre of the gear.
			float originX = body->GetPosition().x;
			float originY = body->GetPosition().y;

			// Point used to position the teeth.
			float pointX = body->GetPosition().x - ( (shape.m_radius) - (tooth_shape.m_radius) );
			float pointY = body->GetPosition().y + (shape.m_radius);
			float weldX = -(shape.m_radius);
			float weldY = 0;
			float x = 0.0f;
			float y = 0.0f;

			for (int i = 0; i < numOfTeeth_A; i++) {
				// Body for the tooth.
				b2BodyDef bd_tooth;
				bd_tooth.type = b2_dynamicBody;

				// Angle of the tooth converted to radians, angle is i * 36°.
				// 36° comes from 360°/10, 10 is the number of teeth.
				angle = ( ((i * 36) * 3.14159265f) / 180.0f );

				// Fixture definition of the tooth.
				b2FixtureDef fd;
				fd.shape = &tooth_shape;
				fd.density = 1.0f;
				fd.friction = 2.0f;

				// Filter settings.
				fd.filter.categoryBits = k_toothCategory;
				fd.filter.maskBits = k_toothMask;

				// Calculating the coordinates of the tooth.
				x = ( ((pointX - originX) * cos(angle)) - ((originY - pointY) * sin(angle)) ) + originX;
				y = ( ((originY - pointY) * cos(angle)) - ((pointX - originX) * sin(angle)) ) + originY;

				bd_tooth.position.Set(x, y);
				b2Body* tooth_body = m_world->CreateBody(&bd_tooth);
				tooth_body->CreateFixture(&fd);

				b2WeldJointDef weld;
				weld.bodyA = body;
				weld.bodyB = tooth_body;
     
				x = 0 + (weldX)*cos(angle) - (weldY)*sin(angle);
				y = 0 + (weldX)*sin(angle) + (weldY)*cos(angle);

				// Connect the centres of the bodies.
				weld.localAnchorA = b2Vec2(x, y);
				weld.localAnchorB = b2Vec2((tooth_shape.m_radius), -(tooth_shape.m_radius));
		
				// Angle of tooth for the weld.
				weld.referenceAngle = angle;
     
				m_world->CreateJoint(&weld);
			}

			// Creating the revolute joint for gear A.
			rjd.Initialize(ground, body, body->GetPosition());
			rjd.motorSpeed = 1.0f;
			rjd.maxMotorTorque = 0.0f;
			rjd.enableMotor = true;
			rjd.collideConnected = false;

			m_revJointA = (b2RevoluteJoint*)m_world->CreateJoint(&rjd);
			
			// Gear B
			b2CircleShape shape3;
			shape3.m_radius = 4.9f;
			b2BodyDef bd3;
			bd3.type = b2_dynamicBody;

			b2RevoluteJointDef rjd2;
			
			bd3.position.Set(3.9f, 13.1f);

			body3 = m_world->CreateBody(&bd3);
			body3->SetGravityScale(0.0f);
			body3->CreateFixture(&shape3, 5.0f);

			// Centre of rotation, being the centre of the gear.
			float originX_2 = body3->GetPosition().x;
			float originY_2 = body3->GetPosition().y;

			// Point used to position the teeth.
			float pointX_2 = body3->GetPosition().x - ( (shape3.m_radius)  - (tooth_shape.m_radius) );
			float pointY_2 = body3->GetPosition().y + ( (shape3.m_radius) );
			float weldX_2 = -(shape3.m_radius);
			float weldY_2 = 0;
			float x_2 = 0.0f;
			float y_2 = 0.0f;

			for (int i = 0; i < numOfTeeth_B; i++) {
				// Body for the tooth.
				b2BodyDef bd_tooth;
				bd_tooth.type = b2_dynamicBody;

				// Angle converted to radians, angle is i * 45°.
				// 45° comes from 360°/8, 8 is the number of teeth.
				angle = ( ((i * 45.0f) * 3.14159265f) / 180.0f );

				// Fixture definition of the tooth.
				b2FixtureDef fd;
				fd.shape = &tooth_shape;
				fd.density = 1.0f;
				fd.friction = 2.0f;

				// Calculating the coordinates of the tooth.
				x_2 = ( ((pointX_2 - originX_2) * cos(angle)) - ((originY_2 - pointY_2) * sin(angle)) ) + originX_2;
				y_2 = ( ((originY_2 - pointY_2) * cos(angle)) - ((pointX_2 - originX_2) * sin(angle)) ) + originY_2;

				bd_tooth.position.Set(x_2, y_2);
				b2Body* tooth_body = m_world->CreateBody(&bd_tooth);
				tooth_body->CreateFixture(&fd);

				b2WeldJointDef weld;
				weld.bodyA = body3;
				weld.bodyB = tooth_body;
     
				x_2 = 0 + (weldX_2)*cos(angle) - (weldY_2)*sin(angle);
				y_2 = 0 + (weldX_2)*sin(angle) + (weldY_2)*cos(angle);

				// Connect the centres of the bodies.
				weld.localAnchorA = b2Vec2(x_2, y_2);
				weld.localAnchorB = b2Vec2((tooth_shape.m_radius), -(tooth_shape.m_radius));
    
				// Angle of tooth for the weld.
				weld.referenceAngle = angle;

				m_world->CreateJoint(&weld);
			}

			rjd2.Initialize(ground, body3, body3->GetPosition());
			rjd2.enableMotor = false;
			rjd2.collideConnected = false;
			m_revJointB = (b2RevoluteJoint*)m_world->CreateJoint(&rjd2);

			// Creating the gear joint
			//b2GearJointDef jointDef;
			//jointDef.bodyA = body;
			//jointDef.bodyB = body3;
			//jointDef.joint1 = m_revJointA;
			//jointDef.joint2 = m_revJointB;
			//jointDef.ratio = 1.0f;// * b2_pi;
			//b2GearJoint* m_GearJoint = (b2GearJoint*)m_world->CreateJoint(&jointDef);
		}

		// Mechanical arms
		{
			// Arm Fixed --------------------------------------
			bd_armFixed.type = b2_dynamicBody;
			bd_armFixed.position.Set(15.0f, 23.0f);
			body_armFixed = m_world->CreateBody(&bd_armFixed);
			body_armFixed->SetGravityScale(0.0f);

			m_armFixed.SetAsBox(5.0f, 0.5f);

			fixDef_armFixed.shape = &m_armFixed;
			fixDef_armFixed.density = 1.0f;

			//Filter settings.
			fixDef_armFixed.filter.categoryBits = k_armCategory;
			fixDef_armFixed.filter.maskBits = k_armMask;

			m_armFixed_fix = body_armFixed->CreateFixture(&fixDef_armFixed);

			pjd_ArmFixed.Initialize(body_armFixed, ground, body_armFixed->GetPosition(), b2Vec2(1, 0));
			pjd_ArmFixed.localAnchorA.Set( 4.0f, 0.0f);
			m_prisArmFixed = (b2PrismaticJoint*)m_world->CreateJoint(&pjd_ArmFixed);

			// Arm C ------------------------------------------
			bd_armC.type = b2_dynamicBody;
			bd_armC.position.Set(15.0f, 17.0f);
			body_armC = m_world->CreateBody(&bd_armC);
			body_armC->SetGravityScale(0.0f);

			m_armC.SetAsBox(0.5f, 7.5f);
			fixDef_armC.shape = &m_armC;
			fixDef_armC.density = 1.0f;

			// Filter settings.
			fixDef_armC.filter.categoryBits = k_armCategory;
			fixDef_armC.filter.maskBits = k_armMask;

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

			m_armA.SetAsBox(9.0f, 0.5f);
			fixDef_armA.shape = &m_armA;
			fixDef_armA.density = 1.0f;

			// Filter 
			fixDef_armA.filter.categoryBits = k_armCategory;
			fixDef_armA.filter.maskBits = k_armMask;
			m_armA_fix = body_armA->CreateFixture(&fixDef_armA);

			//Joint from Arm A to gear A.
			rjd_ArmA_1.bodyA = body;
			rjd_ArmA_1.bodyB = body_armA;
			rjd_ArmA_1.localAnchorB.Set( -8.5f, 0.0f);
			rjd_ArmA_1.localAnchorA.Set(0.0f, 5.0f);
			m_revJointA = (b2RevoluteJoint*)m_world->CreateJoint(&rjd_ArmA_1);

			// Joint from Arm A to Arm C.
			rjd_ArmA_2.Initialize( body_armC, body_armA, body_armC->GetPosition());
			rjd_ArmA_2.localAnchorA.Set( 0.0f, 7.0f);
			rjd_ArmA_2.localAnchorB.Set(8.5f, 0.0f);
			m_revArmC_1 = (b2RevoluteJoint*)m_world->CreateJoint(&rjd_ArmA_2);

			// Arm B ------------------------------------------
			bd_armB.type = b2_dynamicBody;
			bd_armB.position.Set(5.0f, 15.0f);
			body_armB = m_world->CreateBody(&bd_armB);
			body_armB->SetGravityScale(0.0f);

			m_armB.SetAsBox(8.0f, 0.5f);
			fixDef_armB.shape = &m_armB;
			fixDef_armB.density = 1.0f;

			// Filter settings.
			fixDef_armB.filter.categoryBits = k_armCategory;
			fixDef_armB.filter.maskBits = k_armMask;

			m_armB_fix = body_armB->CreateFixture(&fixDef_armB);

			//// Joint from Arm B to wheel B
			rjd_ArmB_1.Initialize(body_armB, body3, body3->GetPosition());
			rjd_ArmB_1.bodyA = body_armB;
			rjd_ArmB_1.bodyB = body3;
			rjd_ArmB_1.localAnchorA.Set( -7.5f, 0.0f);
			rjd_ArmB_1.localAnchorB.Set(0.0f, 4.0f);
			rjd_ArmB_1.enableMotor = false;
			rjd_ArmB_1.collideConnected = false;
			m_revJointB = (b2RevoluteJoint*)m_world->CreateJoint(&rjd_ArmB_1);
			
			//// Joint from Arm B to Arm C.
			rjd_ArmB_2.Initialize(body_armC, body_armB, body_armB->GetPosition());
			rjd_ArmB_2.localAnchorA.Set( 0.0f, -7.0f);
			rjd_ArmB_2.localAnchorB.Set(7.5f, 0.0f);
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
			case 'q':
				if(m_revJointA->IsMotorEnabled()) {
					m_revJointA->EnableMotor(false);
				}
				else {
					m_revJointA->EnableMotor(true);
				}
				break;

			case 'w':
				if (m_revJointA->GetMotorSpeed() < 10) {
					m_revJointA->SetMotorSpeed(m_revJointA->GetMotorSpeed() + 1);
				}
				break;
			case 's':
				if (m_revJointA->GetMotorSpeed() >= 0) {
					m_revJointA->SetMotorSpeed(m_revJointA->GetMotorSpeed() - 1);
				}
				break;

			case 'e':
				if (m_revJointA->GetMaxMotorTorque() < 10000) {
					m_revJointA->SetMaxMotorTorque(m_revJointA->GetMaxMotorTorque() + 10);
				}
				break;
			case 'd':
				if (m_revJointA->GetMaxMotorTorque() >= 1000) {
					m_revJointA->SetMaxMotorTorque(m_revJointA->GetMaxMotorTorque() - 10);
				}
				break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		m_debugDraw.DrawString(5, m_textLine, "Q = start/stop motor, w/s = increase/decrease motor speed, e/d = increase/decrease torque.");
		m_textLine += DRAW_STRING_NEW_LINE;
		m_debugDraw.DrawString(5, m_textLine, "Motor Speed = %4.2f" , m_revJointA->GetMotorSpeed());
		m_textLine += DRAW_STRING_NEW_LINE;
		m_debugDraw.DrawString(5, m_textLine, "Motor torque = %4.2f ", m_revJointA->GetMaxMotorTorque());
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new Mechanical;
	}

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

	int numOfTeeth_A;
	int numOfTeeth_B;
};

#endif