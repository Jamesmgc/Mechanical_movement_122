#ifndef CANNON_H
#define CANNON_H

class Cannon : public Test
{
public:
	b2PolygonShape m_shape1;
	b2PolygonShape m_shape2;
	b2Fixture* m_piece1;
	b2Fixture* m_piece2;
	float angle;
	b2Body* body;
	float radians;
	static const int maxNumber = 250;
	b2Body* m_bodies[maxNumber];

	Cannon()
	{
		angle = 90.0f;
		// Ground
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		// Platform
		{
			float length = 5.0f;
			b2BodyDef bd;
			bd.type = b2_kinematicBody;
			bd.position.Set(5.0f, 7.5f);
			body = m_world->CreateBody(&bd);

			body->SetGravityScale(0.0f);

			b2PolygonShape shape;
			shape.SetAsBox(length, 0.5f);

			b2FixtureDef fd;
			fd.shape = &shape;
			m_platform = body->CreateFixture(&fd);

			m_shape1.SetAsBox(0.5f, 3.0f, b2Vec2(-length, 2.5f), 0.0f);
			m_piece1 = body->CreateFixture(&m_shape1, 1.0f);

			m_shape2.SetAsBox(length, 0.5f, b2Vec2(0,5.0f), 0.0f);
			m_piece2 = body->CreateFixture(&m_shape2, 1.0f);

			radians =  angle * 3.1459f / 180;
		    body->SetTransform( body->GetPosition(),radians);
		}

		//powder
		b2PolygonShape shape;
		shape.SetAsBox(0.1f, 0.1f);

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = 1.0f;
		for (int i = 0; i < maxNumber; ++i)
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;


			bd.position.Set(3.0f,7.0f);
			b2Body* body = m_world->CreateBody(&bd);

			m_bodies[i] = body;

			body->CreateFixture(&fd);
		}

		//cannonBall
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(3.0f, 9.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2CircleShape shape;
			
			shape.m_radius = 1.5f;
			body->CreateFixture(&shape, 20.0f);
		}
	}

	void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
	{
		Test::PreSolve(contact, oldManifold);

		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		/*if (fixtureA == m_platform)
		{
			contact->SetTangentSpeed(5.0f);
		}

		if (fixtureB == m_platform)
		{
			contact->SetTangentSpeed(-5.0f);
		}*/
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
		radians =  angle * 3.1459f / 180;
		body->SetTransform( body->GetPosition(),radians);
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		m_debugDraw.DrawString(5, m_textLine, "Radians = %4.2f" , radians);
		m_textLine += DRAW_STRING_NEW_LINE;
		m_debugDraw.DrawString(5, m_textLine, "Degrees =%4.2f ", angle);
	}

	static Test* Create()
	{
		return new Cannon;
	}
	
	b2Fixture* m_platform;
};

#endif