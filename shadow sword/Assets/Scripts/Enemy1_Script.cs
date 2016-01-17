using UnityEngine;
using System.Collections;

public class Enemy1_Script : MonoBehaviour {
    public int HP;
    public AIPath AI;
    public Animator Enemy_Animator;
    public string Mode;
    private int temp_random;
    public int Reflex_Count;
    public int Reflex_Set;
    public int Skill_ColdDown;
    public int Attack1_ColdDown_Set;
    public int Attack2_ColdDown_Set;
    public int Attack3_ColdDown_Set;
    public float Distance;
    public LayerMask PlayerCheckLayer;
    public Collider[] hitColliders;
    public GameObject Attack1_Pos;
    public float Attack1_Range;
    public int Attack1_Rate;
    public GameObject Attack2_Pos;
    public float Attack2_Speed;
    public int Attack2_Rate;
    public GameObject[] Projectiles;
    public int ATK1;
    public int ATK2;
    public int ATK3;
    private Player_Control player;
    private Projectile_Script projectile_script;
    private GameObject temp_object;
    public bool enable;
    public bool Contact;
    // Use this for initialization
    void Start()
    {
        AI = this.GetComponent<AIPath>();
        Reflex_Count = Reflex_Set;
        Mode = "Sleep";
        AI.target = GameObject.Find("Player").transform;
        AI.canMove = false;
        AI.canSearch = false;
        enable = false;
    }

    // Update is called once per frame
    void Update()
    {
        if (enable == true)
        {
            if (Reflex_Count > 0)
            {
                Reflex_Count -= 1;
            }
            else
            {
                Reflex_Count = Random.Range(Reflex_Set, Reflex_Set * 2);
                temp_random = Random.Range(1, 11);
                if (temp_random == 1)
                {
                    Mode = "Being_Stupid";
                    AI.canMove = false;
                }
                else
                {
                    AI.canMove = true;
                    Distance = Vector3.Distance(this.transform.position, AI.target.position);
                    if (Distance < 20)
                    {
                        Mode = "Close_Range";
                        AI.speed = 200;
                    }
                    else if (Distance < 50)
                    {
                        Mode = "Middle_Range";
                        AI.speed = 200;
                    }
                    else
                    {
                        Mode = "Lost";
                        AI.speed = 80;
                    }
                }
            }


            if (Skill_ColdDown > 0)
                Skill_ColdDown -= 1;
            else
            {
                switch (Mode)
                {
                    case "Being_Stupid":
                        break;
                    case "Close_Range":
                        temp_random = Random.Range(1, 11);
                        if (temp_random < Attack1_Rate)
                        {
                            Enemy_Animator.SetBool("Attack1", true);
                            Skill_ColdDown = Attack1_ColdDown_Set;
                        }

                        break;
                    case "Middle_Range":

                        break;
                    case "Long_Range":

                        break;
                }
            }
        }
    }

    void OnTriggerStay(Collider other)
    {
        if (Contact == true)
        {
            if (other.tag == "Player")
            {
                if (Skill_ColdDown > 0)
                {
                    Skill_ColdDown -= 1;
                }
                else
                {
                    player = other.GetComponent<Player_Control>();
                    player.ReceiveDamage(ATK3);
                    Skill_ColdDown = Attack3_ColdDown_Set;
                }
            }
        }
    }

    void Player_Hit()
    {
        hitColliders = Physics.OverlapSphere(Attack1_Pos.transform.position, Attack1_Range, PlayerCheckLayer);
        if (hitColliders.Length != 0)
        {
            player = hitColliders[0].GetComponent<Player_Control>();
            player.ReceiveDamage(ATK1);
        }
    }

    void Attack2()
    {
        temp_object = (GameObject)Instantiate(Projectiles[0], Attack2_Pos.transform.position, Quaternion.Euler(0, this.transform.rotation.eulerAngles.y, 0));
        projectile_script = temp_object.GetComponent<Projectile_Script>();
        projectile_script.ATK = ATK2;
        projectile_script.Speed = Attack2_Speed;
    }
    void Attack_End()
    {
        Enemy_Animator.SetBool("Attack1", false);
    }

    public void Activate()
    {
        AI = this.GetComponent<AIPath>();
        AI.canMove = true;
        AI.canSearch = true;
        enable = true;
    }
}
