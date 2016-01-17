using System;
using UnityEngine;
using DopplerInteractive.TidyTileMapper.Utilities;

/// <summary>
///A BlockSet - a collection of objects for an orientation of a given block 
/// </summary>
[Serializable]
public class BlockSet
{
	//A blockset is a collection of blocks
	//For a given orientation
	//Parented to a single parent transform 
	//(for the sake of project neatness)
	
	//The parent object
	//This is included for neatness and for
	//Ease of deletion
	
	/// <summary>
	///The object at which this blockset is rooted 
	/// </summary>
	public GameObject rootObject;
	
	//The current display index in the block set
	//When cycling, we increment and wrap this number
	
	/// <summary>
	///The current variation index being displayed for this blockset 
	/// </summary>
	public int currentBlockIndex = 0;
	
	//All blocks in the collection
	
	/// <summary>
	///All of the blocks contained in this blockset 
	/// </summary>
	public GameObject[] blockSet;
	
	//PREFAB
	public GameObject currentObject = null;
	public Vector3 offset = Vector3.zero;
	public Vector3 rotation = Vector3.zero;
	
	//I had much inner turmoil over putting this variable here
	//but from an editor simplicity perspective, it simply makes (sad) sense
	
	[HideInInspector]
	public bool isDefaultSet = false;
	
	#region General Block Show Functions
	
	
	/// <summary>
	///Sets this blockset to be the default for the block containing it 
	/// </summary>
	/// <param name="isDefault">
	///Is this blockset default?
	/// </param>
	public void SetIsDefault(bool isDefault){
		
		isDefaultSet = isDefault;
		
		CleanBlockSet();
	}
	
	/// <summary>
	///Is this blockset the default blockset?
	/// </summary>
	/// <returns>
	///Returns true if default, false if not
	/// </returns>
	public bool IsDefault(){
		return isDefaultSet;
	}
	
	/// <summary>
	///Returns the currently active block from within this blockset 
	/// </summary>
	/// <returns>
	///The gameobject containing the active block
	/// </returns>
	public GameObject GetCurrentBlock(){
		
		if(blockSet.Length == 0){
			return null;
		}
		
		return blockSet[currentBlockIndex];
		
	}
	
	/// <summary>
	///Cycles the active block index of the blockset. Will wrap at 0 and Maximum.
	/// </summary>
	/// <param name="direction">
	///The direction in which the index should be cycled
	/// </param>
	public void CycleIndex(int direction){
				
		if(blockSet.Length <= 0){
			return;
		}
				
		currentBlockIndex += direction;
		
		if(currentBlockIndex < 0){
			currentBlockIndex = blockSet.Length+currentBlockIndex;
		}
		
		if(currentBlockIndex >= blockSet.Length){
			currentBlockIndex = currentBlockIndex - blockSet.Length;
		}
		
		if(blockSet[currentBlockIndex] == null){
			
			CleanBlockSet();
		}
				
	}
	
	public void SetIndex(int index){
		
		if(blockSet.Length <= 0){
			return;
		}
				
		currentBlockIndex = index;
		
		if(currentBlockIndex < 0){
			currentBlockIndex = blockSet.Length+currentBlockIndex;
		}
		
		if(currentBlockIndex >= blockSet.Length){
			currentBlockIndex = currentBlockIndex - blockSet.Length;
		}
		
	}
	
	public void CleanBlockSet(){
		
		if(blockSet == null){
			return;
		}
		
		int length = blockSet.Length;
		
		int nullCount = 0;
		
		for(int i = 0; i < length; i++){
			
			if(blockSet[i] == null){
				nullCount++;
			}
		}
		
		GameObject[] newSet = new GameObject[length - nullCount];
		
		int index = 0;
		
		for(int i = 0; i < length; i++){
			
			if(blockSet[i] == null){
				continue;
			}
			
			newSet[index] = blockSet[i];
			
			index++;
		}
		
		blockSet = newSet;
		
	}
	
	/// <summary>
	///Show the selected block from the blockset 
	/// </summary>
	public void Show(){
		
		Hide();
		
		if(blockSet == null || blockSet.Length <= 0){
			return;
		}
		
		if(currentBlockIndex >= blockSet.Length){
			currentBlockIndex = 0;
		}
				
		if(AssetPool.IsPoolingEnabled()){
			currentObject = AssetPool.Instantiate(blockSet[currentBlockIndex]);
		}
		else{
			currentObject = GameObject.Instantiate(blockSet[currentBlockIndex]) as GameObject;
		}
		
		currentObject.name = blockSet[currentBlockIndex].name;
		
		currentObject.transform.parent = rootObject.transform;
		currentObject.transform.localPosition = offset;
		currentObject.transform.localRotation = Quaternion.Euler(rotation);
		
		//PREFAB
		//blockSet[currentBlockIndex].SetActiveRecursively(true);
				
	}
	
	/// <summary>
	///Hide this blockset 
	/// </summary>
	public void Hide(){
		
		if(currentObject == null){
			return;
		}
		
		if(rootObject == null){
			return;
		}
		
		if(AssetPool.IsPoolingEnabled()){
			
			AssetPool.Destroy(currentObject.gameObject);
			currentObject = null;
			
		}
		else{
			
			if(AssetPool.DestroyImmediate()){			
				GameObject.DestroyImmediate(currentObject.gameObject);
			}
			else{
				GameObject.Destroy(currentObject.gameObject);
			}
			currentObject = null;
			
		}
		
		//PREFAB
		//rootObject.SetActiveRecursively(false);
		//rootObject.active = true;
		
	}
	
	/// <summary>
	///Randomise the active block within the blockset 
	/// </summary>
	public void RandomiseVariant(){
		
		if(blockSet == null){
			return;
		}
		
		if(blockSet.Length <= 0){
			return;
		}
		
		//int random = BlockUtilities.RANDOM.Next(0,blockSet.Length);
		
		int random = UnityEngine.Random.Range(0,blockSet.Length);
		
		CycleIndex(random);
		
		Show();
		
	}
	
	/// <summary>
	///Does this blockset contain any blocks? 
	/// </summary>
	/// <returns>
	///True if the blockset contains blocks, false if not
	/// </returns>
	public bool HasBlocks(){
		
		CleanBlockSet();
		
		if(rootObject == null){
			return false;
		}
		
		if(blockSet == null || blockSet.Length <= 0){
			return false;
		}
		
		return true;
	}
	
	#endregion
	
	#region Editor Functions
	
	/// <summary>
	///Add an object to this blockset 
	/// </summary>
	/// <param name="o">
	///The object you wish to add to the blockset
	/// </param>
	public void Editor_AddToBlockSet(GameObject o, OrientedBlock parent){
				
		int length = blockSet.Length;
		
		for(int i = 0; i < length; i++){
			
			if(blockSet[i] == o){
				return;
			}
		}
		
		this.offset = new Vector3(parent.x_offset,parent.y_offset,parent.z_offset);
		this.rotation = new Vector3(parent.x_rotation,parent.y_rotation,parent.z_rotation);
		
		
		//PREFAB
		/*o.transform.parent = rootObject.transform;
		o.transform.localPosition = offset;
		
		o.transform.localRotation = Quaternion.identity;*/
		
		GameObject[] newSet = new GameObject[length+1];
		
		for(int i = 0; i < length; i++){
			newSet[i] = blockSet[i];
		}
		
		newSet[length] = o;
		
		blockSet = newSet;
		
		//Debug.Log("Adding: " + blockSet.Length);
		
		CleanBlockSet();
		
		//Debug.Log("After clean: " + blockSet.Length);
		
	}
	
	/// <summary>
	///Remove an object from the blockset 
	/// </summary>
	/// <param name="o">
	///The object you wish to remove from the blockset
	/// </param>
	/// <returns>
	///Returns the object you removed from the blockset
	/// </returns>
	public GameObject Editor_RemoveFromBlockSet(GameObject o){
	
		GameObject deletedObject = null;
		
		int length = blockSet.Length;
		
		bool contains = false;
		
		for(int i = 0; i < length; i++){
			
			if(blockSet[i].GetInstanceID() == o.GetInstanceID()){
				contains = true;
			}
		}
		
		if(!contains){
			return null;
		}
		
		GameObject[] newSet = new GameObject[length-1];
		
		int index = 0;
		
		for(int i = 0; i < length; i++){
			
			if(blockSet[i].GetInstanceID() == o.GetInstanceID()){
				deletedObject = blockSet[i];
				continue;
			}
			
			newSet[index] = blockSet[i];
			
			index++;
		}
		
		blockSet = newSet;
		
		CleanBlockSet();
		
		return deletedObject;
	}
		
	#endregion
	
}


