using UnityEngine;
using System.Collections;

public class HP_Bar_Script : MonoBehaviour {
    public Player_Control player;
    private Vector3 HP;
    private Vector3 Position;
	// Use this for initialization
	void Start () {
        HP = new Vector3(player.HP / 10, 10, 1);
        Position = new Vector3(HP.x/2 - 85, 40, 5);
	}
	
	// Update is called once per frame
	void Update () {
        HP.x = player.HP / 10;
        Position.x = HP.x/2 - 85;
      this.transform.localScale = HP;
      this.transform.localPosition = Position;
	}
}
