using UnityEngine;
using System.Collections;

public class Projectile_Script : MonoBehaviour {
    public float Speed;
    public int ATK;
    private Player_Control player;
	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
        this.transform.Translate(Vector3.forward * Speed, Space.Self);
	}

    void OnTriggerEnter(Collider other)
    {
        if(other.tag=="Player")
        {
            player = other.GetComponent<Player_Control>();
            player.ReceiveDamage(ATK);
        }
        Destroy(this.gameObject);
    }
}
