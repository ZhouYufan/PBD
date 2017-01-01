using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class mesh : MonoBehaviour
{
    Mesh Mesh0;
    MeshCollider MeshCollider1;//组成必需collider
    MeshFilter MeshFilter1;//filter
    Vector3[] vertices;//顶点
    Vector3[] velocity;//速度

    GameObject Cube1;

    void Start()
    {

        MeshCollider1 = GetComponent<MeshCollider>();//获取组件
        MeshFilter1 = GetComponent<MeshFilter>();
        Mesh0 = MeshFilter1.mesh;

        GameObject Plane0 = GameObject.CreatePrimitive(PrimitiveType.Plane);//创建plane储存信息
        Mesh Mesh1 = Plane0.GetComponent<MeshFilter>().mesh;

    }
}
	