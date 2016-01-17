using UnityEngine;

public abstract class TidyMapBoundObject : MonoBehaviour
{
	public Block parentBlock;
	public BlockMap parentMap;
	
	public void BindToMap(BlockMap map, Block b){
		this.parentMap = map;
		this.parentBlock = b;
		
		OnBindToMap(map,b);
	}
	
	public virtual void OnBindToMap(BlockMap map, Block b){}
	
	public abstract void OnObjectEnterBlock(Block b,TidyMapBoundObject e);
	public abstract void OnObjectExitBlock(Block b,TidyMapBoundObject e);
	
	public bool destroyWhenStreaming = true;
	
	public virtual bool DestroyWhenStreaming(){
		return destroyWhenStreaming;
	}
}


