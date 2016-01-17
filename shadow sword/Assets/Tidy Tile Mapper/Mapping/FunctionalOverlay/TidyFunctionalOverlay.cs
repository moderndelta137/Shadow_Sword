using System;
using UnityEngine;
using System.Collections.Generic;

public class TidyFunctionalOverlay : MonoBehaviour
{
	//The functional overlay
	//We need to hold this in a list while we edit - we cannot serialize dictionaries
	//After which, we will put it in a hashtable
	public List<TidyFunctionalObject> mapData = new List<TidyFunctionalObject>();
	
	bool isInitialised = false;
	
	Dictionary<Vector3,TidyFunctionalObject> mapDataTable = new Dictionary<Vector3, TidyFunctionalObject>();
	
	public void InitializeFunctionalOverlay(){
		
		isInitialised = true;
		
		for(int i = 0; i < mapData.Count; i++){
			
			Vector3 pos = new Vector3(mapData[i].x,mapData[i].y,mapData[i].depth);
			
			mapDataTable.Add(pos,mapData[i]);
		}
		
		mapData.Clear();
		mapData = null;
	}
	
	public bool HasDataAt(int x, int y, int depth){
		
		Vector3 mapCoords = new Vector3(x,y,depth);
		
		if(!isInitialised){
			
			for(int i = 0; i < mapData.Count; i++){
				if(mapData[i].x == x && mapData[i].y == y && mapData[i].depth == depth){
					return true;
				}
			}
				
			return false;
		}
		else{
			
			return mapDataTable.ContainsKey(mapCoords);
			
		}
		
	}
	
	public TidyFunctionalObject GetDataAt(int x, int y, int depth){
		
		if(!HasDataAt(x,y,depth)){
			return null;
		}
		
		Vector3 mapCoords = new Vector3(x,y,depth);
		
		if(!isInitialised){
			
			for(int i = 0; i < mapData.Count; i++){
				if(mapData[i].x == x && mapData[i].y == y && mapData[i].depth == depth){
					return mapData[i];
				}
			}
		}
		else{
			
			return mapDataTable[mapCoords];
			
		}
		
		return null;
	}
	
	public void AddData(TidyFunctionalObject data){
		
		Vector3 mapCoords = new Vector3(data.x,data.y,data.depth);
		
		if(!isInitialised){
			
			for(int i = 0; i < mapData.Count; i++){
				
				if(mapData[i].x == mapCoords.x && mapData[i].y == mapCoords.y && mapData[i].depth == mapCoords.z){
					
					Debug.LogWarning("Already contained functional overlay data at: " + mapCoords.ToString() + ": removing.");
					
					mapData.RemoveAt(i);
				}
			}
				
			mapData.Add(data);
		}
		else{
			
			if(mapDataTable.ContainsKey(mapCoords)){
				mapDataTable.Remove(mapCoords);
				
				Debug.LogWarning("Already contained functional overlay data at: " + mapCoords.ToString() + ": removing.");
			}
			
			mapDataTable.Add(mapCoords,data);
			
		}
	}
	
	public TidyFunctionalObject RemoveDataAt(int x, int y, int depth){
		Vector3 mapCoords = new Vector3(x,y,depth);
		
		if(!HasDataAt(x,y,depth)){
			return null;
		}
		
		if(!isInitialised){
			
			for(int i = 0; i < mapData.Count; i++){
				
				if(mapData[i].x == x && mapData[i].y == y && mapData[i].depth == depth){
					
					TidyFunctionalObject o = mapData[i];
					
					mapData.RemoveAt(i);
					
					return o;
				}
			}
			
		}
		else{
			
			if(mapDataTable.ContainsKey(mapCoords)){
				
				TidyFunctionalObject o = mapDataTable[mapCoords];
				
				mapDataTable.Remove(mapCoords);
				
				return o;
			}
			
		}
		
		return null;
	}
	
}


