using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Vector3 	gravity = new Vector3(0, -9.8f, 0);

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		for (int tet = 0; tet < tet_number; tet++) {
			inv_Dm [tet] = Build_Edge_Matrix (tet).inverse;
		}
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.
		// Returns the edge matrix of a tetrahedron.
		ret [0, 0] = X [Tet [tet * 4 + 1]].x - X [Tet [tet * 4 + 0]].x;
		ret [1, 0] = X [Tet [tet * 4 + 1]].y - X [Tet [tet * 4 + 0]].y;
		ret [2, 0] = X [Tet [tet * 4 + 1]].z - X [Tet [tet * 4 + 0]].z;
		ret [3, 0] = 1;

		ret [0, 1] = X [Tet [tet * 4 + 2]].x - X [Tet [tet * 4 + 0]].x;
		ret [1, 1] = X [Tet [tet * 4 + 2]].y - X [Tet [tet * 4 + 0]].y;
		ret [2, 1] = X [Tet [tet * 4 + 2]].z - X [Tet [tet * 4 + 0]].z;
		ret [3, 1] = 1;

		ret [0, 2] = X [Tet [tet * 4 + 3]].x - X [Tet [tet * 4 + 0]].x;
		ret [1, 2] = X [Tet [tet * 4 + 3]].y - X [Tet [tet * 4 + 0]].y;
		ret [2, 2] = X [Tet [tet * 4 + 3]].z - X [Tet [tet * 4 + 0]].z;
		ret [3, 2] = 1;

		ret [0, 3] = 0;
		ret [1, 3] = 0;
		ret [2, 3] = 0;
		ret [3, 3] = 1;

		return ret;
	}

	// Compute trace for matrix4x4
	float Matrix4x4_Trace(Matrix4x4 a)
	{
		float ret = 0;
		for (int i = 0; i < 3; i++) {
			ret += a [i, i];
		}	
		return ret;
	}

	// Matrix4x4 add operation
	Matrix4x4 Matrix4x4_Add(Matrix4x4 a, Matrix4x4 b)
	{
		Matrix4x4 ret = Matrix4x4.zero;
		for (int i = 0; i < 3; i++) {
			ret [i, 0] = a [i, 0] + b [i, 0];
			ret [i, 1] = a [i, 1] + b [i, 1];
			ret [i, 2] = a [i, 2] + b [i, 2];
		}
		return ret;
	}

	// Matrix4x4 subtract operation
	Matrix4x4 Matrix4x4_Subtract(Matrix4x4 a, Matrix4x4 b)
	{
		Matrix4x4 ret = Matrix4x4.zero;
		for (int i = 0; i < 3; i++) {
			ret [i, 0] = a [i, 0] - b [i, 0];
			ret [i, 1] = a [i, 1] - b [i, 1];
			ret [i, 2] = a [i, 2] - b [i, 2];
		}
		return ret;
	}

	// Laplacian smoothing
	void laplacianSmoothing()
	{
		// Initial V_sum and V_num
		for (int i = 0; i < number; i++) {
			V_sum [i] = new Vector3 (0, 0, 0);
			V_num [i] = 0;
		}

		// Compute V_sum and V_num
		for (int tet = 0; tet < tet_number; tet++) {
			Vector3 sum = V [Tet [tet * 4 + 0]] + V [Tet [tet * 4 + 1]] + V [Tet [tet * 4 + 2]] + V [Tet [tet * 4 + 3]];
			V_sum [Tet [tet * 4 + 0]] += sum;
			V_sum [Tet [tet * 4 + 1]] += sum;
			V_sum [Tet [tet * 4 + 2]] += sum;
			V_sum [Tet [tet * 4 + 3]] += sum;
			V_num [Tet [tet * 4 + 0]] += 4;
			V_num [Tet [tet * 4 + 1]] += 4;
			V_num [Tet [tet * 4 + 2]] += 4;
			V_num [Tet [tet * 4 + 3]] += 4;
		}

		// Compute V
		for (int i = 0; i < number; i++) {
			V[i] = 0.9f * V[i] + 0.1f*V_sum[i] / V_num[i];
		}
	}

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: Add gravity to Force.
			Force[i] = gravity * mass;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//TODO: Deformation Gradient
			Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];
    		//TODO: Green Strain
			Matrix4x4 G = Matrix4x4.Scale(new Vector3(0.5f, 0.5f, 0.5f)) * Matrix4x4_Subtract(F.transpose * F, Matrix4x4.identity);
    		//TODO: Second PK Stress, Elastic Force
			Matrix4x4 scaler_1 = Matrix4x4.Scale(new Vector3(2f*stiffness_1, 2f*stiffness_1, 2f*stiffness_1));
			float G_trace = Matrix4x4_Trace(G); 
			Matrix4x4 scaler_2 = Matrix4x4.Scale(new Vector3(G_trace*stiffness_0, G_trace*stiffness_0, G_trace*stiffness_0));
			Matrix4x4 S = Matrix4x4_Add(scaler_1 * G, scaler_2 * Matrix4x4.identity);
			//TODO: Update Force
			float scale=-1.0f/(inv_Dm[tet].determinant*6);
			Matrix4x4 temp = F * S * inv_Dm[tet].transpose;
			Force[Tet[tet*4+0]].x-=scale*(temp[0,0]+temp[0,1]+temp[0,2]);
			Force[Tet[tet*4+0]].y-=scale*(temp[1,0]+temp[1,1]+temp[1,2]);
			Force[Tet[tet*4+0]].z-=scale*(temp[2,0]+temp[2,1]+temp[2,2]);
			Force[Tet[tet*4+1]].x+=scale*temp[0,0];
			Force[Tet[tet*4+1]].y+=scale*temp[1,0];
			Force[Tet[tet*4+1]].z+=scale*temp[2,0];
			Force[Tet[tet*4+2]].x+=scale*temp[0,1];
			Force[Tet[tet*4+2]].y+=scale*temp[1,1];
			Force[Tet[tet*4+2]].z+=scale*temp[2,1];
			Force[Tet[tet*4+3]].x+=scale*temp[0,2];
			Force[Tet[tet*4+3]].y+=scale*temp[1,2];
			Force[Tet[tet*4+3]].z+=scale*temp[2,2];
    	}

		laplacianSmoothing();
    	for(int i=0; i<number; i++)
    	{
    		//TODO: Update X and V here.
			V [i] += (Force [i] * dt / mass) * damp;
			X [i] += V [i] * dt;

    		//TODO: (Particle) collision with floor.
    		if(X[i].y < -3.0f)
    		{
    			V[i].x = 0;
    			V[i].z = 0;
    			V[i].y += (-3.0f - X[i].y) / dt;
    			X[i].y = -3.0f;
    		}
    	}
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }
}
