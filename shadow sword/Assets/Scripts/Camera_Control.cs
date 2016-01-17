using UnityEngine;
using System.Collections;

public class Camera_Control : MonoBehaviour {
    public Transform Player;
	// Use this for initialization
	void Start () {
	}
	
	// Update is called once per frame
	void Update () {
        this.transform.position = new Vector3(Player.transform.position.x, 40, Player.transform.position.z);
	}
}
