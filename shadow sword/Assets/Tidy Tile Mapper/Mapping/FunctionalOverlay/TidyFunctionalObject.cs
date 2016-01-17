using System;
using UnityEngine;

public class TidyFunctionalObject : MonoBehaviour{
	
	public Block parentBlock;
	
	public int x{
		get { return parentBlock.x; }
	}
	
	public int y{
		get { return parentBlock.y; }
	}
	
	public int depth{
		get { return parentBlock.depth; }
	}
	
	public void SetParentBlock(Block parentBlock){
		this.parentBlock = parentBlock;
	}
	
}
	


