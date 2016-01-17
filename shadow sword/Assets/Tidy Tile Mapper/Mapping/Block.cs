using System;
using UnityEngine;
using DopplerInteractive.TidyTileMapper.Utilities;

/// <summary>
///The base class for a Block 
/// </summary>
public abstract class Block : MonoBehaviour
{
	/// <summary>
	///The x and y map coordinates of this block. 
	/// </summary>
	public int x, y, depth;
	
	/// <summary>
	///The map to which this block is bound 
	/// </summary>
	public BlockMap blockMap;
	
	//Is this a null block?
	//We use this to... symbolise emptiness
	//In the block editor
	//It will be cleared when building
	
	/// <summary>
	///Should this block act as an empty block when being treated by map logic? 
	/// </summary>
	public bool actAsEmptyBlock = false;
	/// <summary>
	///Is this a null working block? 
	/// </summary>
	public bool isNullBlock = false;
	/// <summary>
	///Should this block retain its working collider when previewed or published? 
	/// </summary>
	public bool retainCollider = true;
	/// <summary>
	///Should this block disappear (remove its renderer) but stay active (retain its collider)
	///When published or previewed? This is used to create new triggers from plugins, mostly
	/// </summary>
	public bool disappearButStayActive = false;
		
	/// <summary>
	///Return the default block for this Block. 
	/// </summary>
	/// <returns>
	/// Returns a prefab reference to the default block
	/// </returns>
	public abstract GameObject GetDefaultBlock();
	
	/// <summary>
	///Binds this block to its parent map, and sets its initial x,y coords. 
	/// </summary>
	/// <param name="x">
	/// Initial x coordinate for this block
	/// </param>
	/// <param name="y">
	/// Initial y coordinate for this block
	/// </param>
	/// <param name="blockMap">
	/// The blockmap to which this block will be bound
	/// </param>
	public void BindToMap(int x, int y, int depth, BlockMap blockMap){
	
		this.x = x;
		this.y = y;
		this.depth = depth;
		this.blockMap = blockMap;
				
		OnBindToMap(x,y,depth,blockMap);
		
		BindChildrenToBlock();
	}
	
	public void BindChildrenToBlock(){
		
		TidyMapBoundObject[] boundObjects = gameObject.GetComponentsInChildren<TidyMapBoundObject>();
		
		for(int i = 0; i < boundObjects.Length; i++){
			boundObjects[i].BindToMap(blockMap,this);
		}
		
	}
	
	public virtual void OnBindToMap(int x, int y, int depth, BlockMap blockMap){}
	
	/// <summary>
	/// Refreshes the block.
	/// </summary>
	public void RefreshBlock(){
		
		OnRefreshBlock();
		
		BindChildrenToBlock();
		
	}
	
	public virtual void OnRefreshBlock(){}
	
	/// <summary>
	///Show the block 
	/// </summary>
	public void Show(){
		
		//gameObject.SetActiveRecursively(true);
		
		OnShow();
	}
	
	public virtual void OnShow(){}
	
	/// <summary>
	///Hide the block 
	/// </summary>
	public void Hide(){
		
		//gameObject.SetActiveRecursively(false);
		
		//gameObject.active = true;
		
		OnHide();
		
	}
	
	public virtual void OnHide(){}
	
	public virtual void RandomiseVariant(){
		
	}
	
	public virtual void SetVariant(int variant){
		
	}
	
	/// <summary>
	///Clear the Default block - largely used by Editor functions. 
	/// </summary>
	public virtual void ClearDefault(){
		
		return;
		
	}
	
	public virtual void ObjectEnteredBlock(TidyMapBoundObject e){}
	
	public virtual void ObjectExitedBlock(TidyMapBoundObject e){}
	
}

