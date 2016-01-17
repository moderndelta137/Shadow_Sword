using UnityEngine;
using System.Collections;

public class Player_Control : MonoBehaviour {
    public float Move_Speed;
    public int HP;
    public int ATK;
    public int DEF;
    public Vector3 Mouse_Pos;
    public Camera MainCamera;
    public Animator Player_Animator;
    public Transform Sword_Trans;
    public float Attack_Range;
    public int Fire_Check_Range;
    public LayerMask EnemyCheckLayer;
    public LayerMask FireCheckLayer;
    public LayerMask SpawnerCheckLayer;
    public Collider[] hitColliders;
    public Collider[] enemyhisColliders;
    public Collider[] spawnerhisColliders;
    public GameObject Fire_Prefab;
    public GameObject Light_Prefab;
    public ParticleSystem Sword_Particle;
    public Light Sowrd_Light;
    private int array_index;
    private float distance;
    private Dragon_Script dragon;
    private Enemy1_Script enemy;
    private Spawner_Script spawner;
    private Rigidbody rb;
    public AudioSource Hit_SFX;
    public AudioSource Damage_SFX;
	// Use this for initialization
	void Start () {
        rb = this.GetComponent<Rigidbody>();
	}
    void FixedUpdate()
    {
        rb.velocity = Vector3.zero;
    }
	// Update is called once per frame
    void Update()
    {

            //Rotate Player Towards the Mouse_Pos
            Mouse_Pos = MainCamera.ScreenToWorldPoint(Input.mousePosition);
            Mouse_Pos.y = 10;
            transform.LookAt(Mouse_Pos, Vector3.up);

            //Move Player with WASD
            /*
            if (Input.GetButton("Horizontal"))
            {
                transform.Translate(Vector3.right * Move_Speed * Input.GetAxisRaw("Horizontal"), Space.Self);
            }*/
            if (Input.GetButton("Vertical"))
            {
                transform.Translate(Vector3.forward * Move_Speed * Input.GetAxisRaw("Vertical"), Space.Self);
            }

            //Attack with Left_Click
            if (Input.GetButtonDown("Fire1"))
            {
                Player_Animator.SetBool("Attack", true);
            }
            CheckForFires();
            enemyhisColliders = Physics.OverlapSphere(this.transform.position, 80, EnemyCheckLayer);
            if (enemyhisColliders.Length != 0)
            {
                for (array_index = 0; array_index < enemyhisColliders.Length; array_index++)
                {
                    if (enemyhisColliders[array_index].tag == "Dragon")
                    {

                    }
                    else
                    {
                        enemy = enemyhisColliders[array_index].GetComponent<Enemy1_Script>();
                        enemy.Activate();
                    }
                }
            }
            spawnerhisColliders = Physics.OverlapSphere(this.transform.position, 80, SpawnerCheckLayer);
            if (spawnerhisColliders.Length != 0)
            {
                for (array_index = 0; array_index < spawnerhisColliders.Length; array_index++)
                {

                    spawner = spawnerhisColliders[array_index].GetComponent<Spawner_Script>();
                    spawner.Activate = true;
                }
            }
        
    }

    void CheckForFires()
    {
        hitColliders = Physics.OverlapSphere(transform.position, Fire_Check_Range, FireCheckLayer);
        ATK=100;
        DEF = 0;
        Sword_Particle.emissionRate = 20;
        Sowrd_Light.intensity = 8;
        if (hitColliders.Length != 0)
        {
            for (array_index = 0; array_index < hitColliders.Length; array_index++)
            {
                distance = Vector3.Distance(transform.position, hitColliders[array_index].transform.position);
                if(distance<10)
                {
                    ATK -= 50;
                    DEF += 30;
                    Sword_Particle.emissionRate -= 10;
                    Sowrd_Light.intensity -= 4;
                }
                else if (distance < 20)
                {
                    ATK -= 30;
                    DEF += 10;
                    Sword_Particle.emissionRate -= 5;
                    Sowrd_Light.intensity -= 3;
                }
                else if (distance<30)
                {
                    ATK -= 10;
                    DEF += 5;
                    Sword_Particle.emissionRate -= 3;
                    Sowrd_Light.intensity -= 2;
                }
                else 
                {
                    ATK -= 5;
                    DEF += 1;
                    Sword_Particle.emissionRate -= 1;
                    Sowrd_Light.intensity -= 1;
                }
            }
            if (ATK < 10)
                ATK = 10;
            if (DEF > 100)
                DEF = 100;
            if (Sword_Particle.emissionRate < 0)
                Sword_Particle.emissionRate = 0;
            if (Sowrd_Light.intensity < 0)
                Sowrd_Light.intensity = 0;
        }
    }

    void AttackHit()
    {
       hitColliders = Physics.OverlapSphere(Sword_Trans.position, Attack_Range, EnemyCheckLayer);
       if (hitColliders.Length != 0)
       {
           Hit_SFX.Play();
           for (array_index = 0; array_index < hitColliders.Length; array_index++)
           {
               if (hitColliders[array_index].tag == "Dragon")
               {
                   dragon = hitColliders[array_index].GetComponent<Dragon_Script>();
                   dragon.HP -= ATK;
                   if (dragon.HP <= 0)
                   {
                       Destroy(hitColliders[array_index].gameObject);
                       MainCamera.GetComponent<FOWEffect>().enabled = false;
                       Instantiate(Light_Prefab);
                   }
               }
               else
               {
                   enemy = hitColliders[array_index].GetComponent<Enemy1_Script>();
                   enemy.HP -= ATK;
                   if (enemy.HP <= 0)
                   {
                       Destroy(hitColliders[array_index].gameObject);
                       Instantiate(Fire_Prefab, hitColliders[array_index].transform.position, Quaternion.identity);
                   }
               }
           }
       }
    }
    void AttackEnd()
    {
        Player_Animator.SetBool("Attack", false);
    }


    public void ReceiveDamage(int ATK)
    {
        HP -= ATK;
        Damage_SFX.Play();
        if (HP <= 0)
        {
            Destroy(this.gameObject);
            Application.LoadLevel("gameover");
            //GameOver
        }
    }
}
