using UnityEngine;
using System.Collections;

public class Dragon_Script : MonoBehaviour {
    public int HP;
    public AIPath AI;
    public Animator Dragon_Animator;
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
    public GameObject Attack2_Pos;
    public float Attack2_Speed;
    public GameObject[] Projectiles;
    public int ATK1;
    public int ATK2;
    public int ATK3;
    private Player_Control player;
    private Projectile_Script projectile_script;
    private GameObject temp_object;
    public AudioSource Attack_SFX;
    public AudioSource Roar_SFX;
	// Use this for initialization
	void Start () {
        AI = this.GetComponent<AIPath>();
        Reflex_Count = Reflex_Set;
	}
	
	// Update is called once per frame
	void Update () {

        if (Reflex_Count > 0)
        {
            Reflex_Count -= 1;
        }
        else
        {
            Reflex_Count = Random.Range(Reflex_Set,Reflex_Set*2);
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
                if (Distance < 40)
                {
                    Mode = "Close_Range";
                    AI.speed = 50;
                }
                else if (Distance < 70)
                {
                    Mode = "Middle_Range";
                    AI.speed = 100;
                }
                else
                {
                    Mode = "Long_Range";
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
                    if (temp_random < 5)
                    {
                        Dragon_Animator.SetBool("Attack1", true);
                        Skill_ColdDown = Attack1_ColdDown_Set;  
                    }
                    else if (temp_random < 8)
                    {
                        Dragon_Animator.SetBool("Attack2", true);
                        Skill_ColdDown = Attack2_ColdDown_Set;
                    }
                    else if (temp_random == 9)
                    {
                        Dragon_Animator.SetBool("Attack3", true);
                        Skill_ColdDown = Attack3_ColdDown_Set;
                    }
                    break;
                case "Middle_Range":
                    temp_random = Random.Range(1, 11);
                    if (temp_random < 5)
                    {
                        Dragon_Animator.SetBool("Attack2", true);
                        Skill_ColdDown = Attack2_ColdDown_Set;
                    }
                    else if (temp_random == 7)
                    {
                        Dragon_Animator.SetBool("Attack3", true);
                        Skill_ColdDown = Attack3_ColdDown_Set;
                    }
                    break;
                case "Long_Range":
                    temp_random = Random.Range(1, 11);
                    if (temp_random < 3)
                    {
                        Dragon_Animator.SetBool("Attack2", true);
                        Skill_ColdDown = Attack2_ColdDown_Set;
                    }
                    else if (temp_random < 6)
                    {
                        Dragon_Animator.SetBool("Attack3", true);
                        Skill_ColdDown = Attack3_ColdDown_Set;
                    }
                    break;
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
        Attack_SFX.Play();
        temp_object = (GameObject)Instantiate(Projectiles[0], Attack2_Pos.transform.position, Quaternion.Euler(0,this.transform.rotation.eulerAngles.y,0));
        projectile_script = temp_object.GetComponent<Projectile_Script>();
        projectile_script.ATK = ATK2;
        projectile_script.Speed = Attack2_Speed;
    }
    void Attack3()
    {
        Roar_SFX.Play();
        temp_object = (GameObject)Instantiate(Projectiles[0], Attack2_Pos.transform.position, Quaternion.Euler(0, this.transform.rotation.eulerAngles.y, 0));
        projectile_script = temp_object.GetComponent<Projectile_Script>();
        projectile_script.ATK = ATK2;
        projectile_script.Speed = Attack2_Speed;
        temp_object = (GameObject)Instantiate(Projectiles[0], Attack2_Pos.transform.position, Quaternion.Euler(0, this.transform.rotation.eulerAngles.y + 30, 0));
        projectile_script = temp_object.GetComponent<Projectile_Script>();
        projectile_script.ATK = ATK2;
        projectile_script.Speed = Attack2_Speed;
        temp_object = (GameObject)Instantiate(Projectiles[0], Attack2_Pos.transform.position, Quaternion.Euler(0, this.transform.rotation.eulerAngles.y - 30, 0));
        projectile_script = temp_object.GetComponent<Projectile_Script>();
        projectile_script.ATK = ATK2;
        projectile_script.Speed = Attack2_Speed;
    }
    void Attack_End()
    {
        Dragon_Animator.SetBool("Attack1", false);
        Dragon_Animator.SetBool("Attack2", false);
        Dragon_Animator.SetBool("Attack3", false);
    }
}
