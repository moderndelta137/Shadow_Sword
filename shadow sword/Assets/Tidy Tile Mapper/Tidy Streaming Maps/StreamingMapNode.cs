using System;
using UnityEngine;
using System.Collections.Generic;

[Serializable]
public class StreamingMapNode
{
	public Block bPrefab;
	
	public Block blockPrefab{
		get{
			return bPrefab;
		}
		set{
			bPrefab = value;
			vIndex = -1;
		}
	}
	
	public Material backgroundEntry;
	
	public int vIndex;
	
	public int variantIndex{
		get{
			if(vIndex < 0){
				return 0;
			}
			
			return vIndex;
		}
		set{
								
			vIndex = value;
		}
	}
	
	public bool HasVariant(){
		
		return (variantIndex >= 0);
		
	}
	
	public StreamingMapNode (Block block) : this(block,-1,null){}
	
	public StreamingMapNode (Block block, int variantIndex) : this(block,variantIndex,null){}
	
	public StreamingMapNode (Material background) : this(null,-1,null){}
	
	public StreamingMapNode(Block block, int variantIndex, Material background){
		
		this.blockPrefab = block;
		this.variantIndex = variantIndex;
		this.backgroundEntry = background;
		
	}
	
	public StreamingMapNode(string saveString, char delimiter, Dictionary<string,Block> blockDictionary, Dictionary<string,Material> materialDictionary){
		
		string[] sEntries = saveString.Split(delimiter);
		
		if(sEntries.Length < 3){
			return;
		}
		
		if(blockDictionary.ContainsKey(sEntries[0])){
			blockPrefab = blockDictionary[sEntries[0]];
		}
		
		if(materialDictionary.ContainsKey(sEntries[1])){
			backgroundEntry = materialDictionary[sEntries[1]];
		}
		
		if(!int.TryParse(sEntries[2],out vIndex)){
			variantIndex = -1;
		}
	
		OnLoadFromString(sEntries,blockDictionary,materialDictionary);
		
	}
	
	public virtual void OnLoadFromString(string[] saveEntries, Dictionary<string,Block> blockDictionary, Dictionary<string,Material> materialDictionary){
	}
	
	public string ToSaveString(char delimiter){
		
		return ((blockPrefab==null) ? "" : blockPrefab.name) + delimiter + ((backgroundEntry == null) ? "" : backgroundEntry.name) + delimiter + vIndex + delimiter + OnToSaveString(delimiter);
		
	}
	
	public virtual string OnToSaveString(char delimiter){
		return "";
	}
}


