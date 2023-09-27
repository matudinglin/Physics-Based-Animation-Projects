using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour
{
	bool launched = false;
	float dt = 0.015f;
	Vector3 v = new Vector3(0, 0, 0);   // velocity
	Vector3 w = new Vector3(0, 0, 0);   // angular velocity
	Vector3 g = new Vector3(0, -9.8f, 0);   // gravity

	float mass;                                 // mass
	Matrix4x4 I_ref;                            // reference inertia

	float linear_decay = 0.999f;                // for velocity decay
	float angular_decay = 0.98f;
	float restitution = 0.5f;                   // for collision


	// Use this for initialization
	void Start()
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m = 1;
		mass = 0;
		for (int i = 0; i < vertices.Length; i++)
		{
			mass += m;
			float diag = m * vertices[i].sqrMagnitude;
			I_ref[0, 0] += diag;
			I_ref[1, 1] += diag;
			I_ref[2, 2] += diag;
			I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
			I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
			I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
			I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
			I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
			I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
			I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
			I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
			I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
		}
		I_ref[3, 3] = 1;
	}

	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A[0, 0] = 0;
		A[0, 1] = -a[2];
		A[0, 2] = a[1];
		A[1, 0] = a[2];
		A[1, 1] = 0;
		A[1, 2] = -a[0];
		A[2, 0] = -a[1];
		A[2, 1] = a[0];
		A[2, 2] = 0;
		A[3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		Vector3 x = transform.position;

		// Get sum of ri
		Vector3 riSum = new Vector3(0, 0, 0);
		int counter = 0;
		Vector3[] vertices = GetComponent<MeshFilter>().mesh.vertices;
		for (int i = 0; i < vertices.Length; i++)
		{
			Vector3 ri = R * vertices[i];
			Vector3 xi = x + ri;
			Vector3 vi = v + Vector3.Cross(w, ri);

			if (Vector3.Dot(xi - P, N) < 0 && Vector3.Dot(vi, N) < 0)
			{
				riSum += ri;
				counter++;
			}
		}

		// If there is at least one vertex collide with the plane
		if (counter > 0)
		{
			Vector3 riAvg = riSum / counter;
			Vector3 viAvg = v + Vector3.Cross(w, riAvg);
			if (Mathf.Abs(viAvg.y + 9.8f * dt) < 4.0f * dt) restitution = 0;
			Matrix4x4 inverseI = R * I_ref.inverse * R.transpose;

			// Get K matrix
			Matrix4x4 Rstar = Get_Cross_Matrix(riAvg);
			Matrix4x4 K = Rstar.transpose * inverseI * Rstar;
			K[0, 0] += 1.0f / mass;
			K[1, 1] += 1.0f / mass;
			K[2, 2] += 1.0f / mass;

			// Calculate Impulse
			Vector3 temp = -viAvg - restitution * viAvg.y * N;
			Vector3 j = K.inverse * temp;

			// Update v and w
			v = v + j / mass;
			w = w + (Vector3)(inverseI * Vector3.Cross(riAvg, j));
		}
	}

	// Quaternion Addtion
	Quaternion quaterAddtion(Quaternion q1, Quaternion q2)
	{
		Quaternion q = new Quaternion(0, 0, 0, 0);
		q.x = q1.x + q2.x;
		q.y = q1.y + q2.y;
		q.z = q1.z + q2.z;
		q.w = q1.w + q2.w;
		return q;
	}

	// Update is called once per frame
	void Update()
	{
		//Game Control
		if (Input.GetKey("r"))
		{
			transform.position = new Vector3(0, 0.6f, 0);
			restitution = 0.5f;
			launched = false;
		}
		if (Input.GetKey("l"))
		{
			v = new Vector3(5, 2, 0);
			w = new Vector3(0, 0, 0);
			launched = true;
		}

		if (!launched)
			return;

		// Update velocities and angular velocities
		v = linear_decay * (v + g * dt);
		w = angular_decay * w;

		// Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Update position & orientation
		Vector3 x = transform.position;
		Quaternion q = transform.rotation;

		x = x + v * dt;
		Vector3 temp = w * dt * 0.5f;
		Quaternion tq = new Quaternion(temp.x, temp.y, temp.z, 0);
		q = quaterAddtion(q, tq * q);
		q = q.normalized;

		transform.position = x;
		transform.rotation = q;
	}
}
