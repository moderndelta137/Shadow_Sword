using UnityEngine;
using System.Collections;

public class Fire_Script : MonoBehaviour {
    public LayerMask SpawnerCheckLayer;
    public int Spawner_Check_Range;
    public Collider[] hitColliders;
    private int array_index;
    private Spawner_Script spawner_script;
	// Use this for initialization
	void Start () {
        hitColliders = Physics.OverlapSphere(transform.position, Spawner_Check_Range, SpawnerCheckLayer);
        if (hitColliders.Length != 0)
        {
            for (array_index = 0; array_index < hitColliders.Length; array_index++)
            {
                spawner_script = hitColliders[array_index].GetComponent<Spawner_Script>();
                spawner_script.Spawner_Enable = false;
            }
        }
	}
	
	// Update is called once per frame
	void Update () {
	
	}

    void OnDestroy()
    {
        hitColliders = Physics.OverlapSphere(transform.position, Spawner_Check_Range, SpawnerCheckLayer);
        if (hitColliders.Length != 0)
        {
            for (array_index = 0; array_index < hitColliders.Length; array_index++)
            {
                spawner_script = hitColliders[array_index].GetComponent<Spawner_Script>();
                spawner_script.Spawner_Enable = true;
            }
        }
    }
}
