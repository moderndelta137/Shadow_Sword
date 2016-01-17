using UnityEngine;
using System.Collections;

public class Spawner_Script : MonoBehaviour {
    public bool Spawner_Enable;
    public bool Activate;
    public int ColdDown;
    public int ColdDown_Set;
    public GameObject[] Monsters;
    private int temp_random;
	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
    void Update()
    {
        if (Spawner_Enable == true)
        {
            if (Activate == true)
            {
                if (ColdDown > 0)
                    ColdDown -= 1;
                else
                {
                    temp_random = Random.Range(0, 10);

                    Instantiate(Monsters[temp_random], this.transform.position, Quaternion.identity);
                    ColdDown = Random.Range(ColdDown_Set, ColdDown_Set * 2);
                }
            }
        }
    }
}
