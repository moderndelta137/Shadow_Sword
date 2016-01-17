using UnityEngine;
using System.Collections;

public class Trap_Script : MonoBehaviour {
    public LayerMask FireCheckLayer;
    public int Fire_Check_Range;
    public Collider[] hitColliders;
    private int array_index;
    private BoxCollider collider;
    public AudioSource Trap_SFX;
	// Use this for initialization
	void Start () {
        collider = gameObject.GetComponent<BoxCollider>();
	}
	
	// Update is called once per frame
	void Update () {
	
	}
    void OnTriggerEnter()
    {
        collider.isTrigger = false;
        collider.enabled = false;
        Trap_SFX.Play();
        hitColliders = Physics.OverlapSphere(transform.position, Fire_Check_Range, FireCheckLayer);
        if (hitColliders.Length != 0)
        {
            for (array_index = 0; array_index < hitColliders.Length; array_index++)
            {
                Destroy(hitColliders[array_index].gameObject);
            }
        }
    }
}
