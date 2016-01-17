using UnityEngine;
using System.Collections;

public class Opening : MonoBehaviour {
    public AudioSource Dragon;
	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
        if (Input.anyKeyDown)
        {
            Application.LoadLevel("Dungeon");
        }
	}
    void DragonRoar()
    {
        Dragon.Play();
    }
    void EndScene()
    {
        Application.LoadLevel("Dungeon");
    }
}
