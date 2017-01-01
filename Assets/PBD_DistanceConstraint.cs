using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class PBD_DistanceConstraint : MonoBehaviour {

	int i;
	public int solverIterations=20;
	public Vector3 gravity=new Vector3(0,0,0);
	[Range(0f,0.1f)]public float damping;
	public List<Constraints> ConnectList;
	public float[] m;
	float[] w;

	Vector3[] Position, velocities;
	public Transform[] vertices;

	void Start () {
		Position = new Vector3[vertices.Length];
		velocities = new Vector3[vertices.Length];
		w = new float[vertices.Length];

		for (i = 0; i < vertices.Length; i++) 
		{
			velocities [i] = new Vector3(0,0,0);
			Position [i] = new Vector3(0,0,0);
			w [i] = 1.0f / m [i];
		}
	}
	

	void Update () 
	{
		for (i = 0; i < vertices.Length; i++) //line5有外力的状况下
		{
			velocities[i]+=Time.deltaTime*w[i]* Force(vertices[i].position);
	    }
		dampVelocities (velocities);//line6阻尼(3.5)
		for(i=0;i<vertices.Length;i++)
		{
			Position[i]=vertices[i].position+Time.deltaTime*velocities[i];//line7:p=x+vt
		}
//		for (i = 0; i < vertices.Length; i++) //line8
//		{
//			generateCollisionConstraints (Position [i], vertices [i].position);
//		}
		for(i=0;i<vertices.Length;i++)//line9-11调用ProjectConstraints约束投影得出p 迭代次数自定20
		{
			projectConstraints (Position);
		}
		for (i = 0; i < vertices.Length; i++) //line13-14更新速度和位置
		{
			velocities [i] = (Position [i] - vertices [i].position) / Time.deltaTime;
			vertices [i].position = Position [i];
		}
}

	Vector3 Force(Vector3 p)   //外力（重力）
	{
		return gravity;
	}

	void dampVelocities(Vector3[] v)   //阻尼函数
	{
		Vector3 sumX = Vector3.zero, sumV = Vector3.zero;
		float sumM = 0;
		for (int i = 0; i < v.Length; i++)
		{
			sumX += vertices[i].position * m[i];
			sumV += v[i] * m[i];
			sumM += m[i];
		}
		Vector3 centerX = sumX / sumM;
		Vector3 centerV = sumV / sumM;

		Vector3 tempL = Vector3.zero;
		Matrix4x4 tempI = Matrix4x4.zero;
		for (int i = 0; i < v.Length; i++)
		{
			Vector3 r = vertices[i].position - centerX;
			tempL += Vector3.Cross(r, m[i] * v[i]);

			Matrix4x4 rm = new Matrix4x4();
			rm.SetColumn(0, r);
			tempI = MatrixAdd(tempI, MatrixMultiply( rm * rm.transpose,m[i]));//调用MatrixAdd MatrixMultiply
		}
		Vector3 w = tempI.inverse.MultiplyVector(tempL) ;
		for(int i = 0; i < v.Length; i++)
		{
			Vector3 r = vertices[i].position - centerX;
			Vector3 deltaV = centerV + Vector3.Cross(w, r) - v[i];
			v[i] += damping * deltaV;
		}

	}
	static Matrix4x4 MatrixAdd(Matrix4x4 a,Matrix4x4 b)
	{
		Matrix4x4 m = Matrix4x4.zero;
		m.m00 = a.m00 + b.m00;
		m.m01 = a.m01 + b.m01;
		m.m02 = a.m02 + b.m02;
		m.m10 = a.m10 + b.m10;
		m.m11 = a.m11 + b.m11;
		m.m12 = a.m12 + b.m12;
		m.m20 = a.m20 + b.m20;
		m.m21 = a.m21 + b.m21;
		m.m22 = a.m22 + b.m22;
		return m;
	}

	static Matrix4x4 MatrixMultiply(Matrix4x4 a, float b)
	{
		Matrix4x4 m = Matrix4x4.zero;
		m.m00 = a.m00 * b;
		m.m01 = a.m01 * b;
		m.m02 = a.m02 * b;
		m.m10 = a.m10 * b;
		m.m11 = a.m11 * b;
		m.m12 = a.m12 * b;
		m.m20 = a.m20 * b;
		m.m21 = a.m21 * b;
		m.m22 = a.m22 * b;
		return m;
	}

	[System.Serializable]
	public class Constraints
	{
		public int n1, n2;
		public float Mydist;
		public bool type;
		public float k;
	}
	void projectConstraints(Vector3[] p)//投影约束函数 
	{
		foreach (var constraint in ConnectList)
		{
			int i1 = constraint.n1;
			int i2 = constraint.n2;
			Vector3 deltaP1 = -w[i1] / (w[i1] + w[i2]) * (Vector3.Magnitude(p[i1] - p[i2]) - constraint.Mydist) * (p[i1] - p[i2]) / Vector3.Magnitude(p[i1] - p[i2]);
			Vector3 deltaP2 = w[i2] / (w[i1] + w[i2]) * (Vector3.Magnitude(p[i1] - p[i2]) - constraint.Mydist) * (p[i1] - p[i2]) / Vector3.Magnitude(p[i1] - p[i2]);
			p[i1] += deltaP1;
			p[i2] += deltaP2;
		}
	}

}