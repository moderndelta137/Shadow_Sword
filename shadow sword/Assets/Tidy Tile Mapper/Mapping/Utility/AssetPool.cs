using System;
using UnityEngine;
using System.Collections.Generic;

namespace DopplerInteractive.TidyTileMapper.Utilities
{
		
	public class AssetPool
	{
		static bool USE_RESOURCES = false;
		
		public static void EnableResources(){
			USE_RESOURCES = true;
		}
		
		public static void DisableResources(){
			USE_RESOURCES = false;
		}
		
		static bool POOLING_ENABLED = false;
		static bool DESTROY_IMMEDIATE = true;
		
		public static bool DestroyImmediate(){
			return DESTROY_IMMEDIATE;
		}
		
		public static void EnableDestroyImmediate(){
			DESTROY_IMMEDIATE = true;
		}
		
		public static void DisableDestroyImmediate(){
			DESTROY_IMMEDIATE = false;
		}
		
		static GameObject assetPoolObject = null;
		
		static AssetPoolEntry compEntry = new AssetPoolEntry(null);
		
		public static void EnablePooling(){
			POOLING_ENABLED = true;
		}
		
		public static void DisablePooling(){
			POOLING_ENABLED = false;
		}
		
		public static bool IsPoolingEnabled(){
			return POOLING_ENABLED;
		}
		
		static bool expireAssets = false;
		static float assetLifeTime = 0.0f;
		
		public static void EnableAssetExpiry(float timeToLife){
			expireAssets = true;
			assetLifeTime = timeToLife;
		}
		
		public static void DisableAssetExpiry(){
			expireAssets = false;
		}
			
		public static void UpdatePool(float deltaTime){
			
			if(!expireAssets){
				return;
			}
			
			foreach(string key in pool.Keys){
				
				List<AssetPoolEntry> entries = pool[key];
				
				for(int i = 0; i < entries.Count; i++){
					
					entries[i].lifeTime += deltaTime;
					
					if(entries[i].lifeTime >= assetLifeTime){
						
						if(DESTROY_IMMEDIATE){
							GameObject.DestroyImmediate(entries[i].gameObject);
						}
						else{
							GameObject.Destroy(entries[i].gameObject);
						}
						
						entries[i] = null;
						
						entries.RemoveAt(i);
						
						i--;
						
					}
					
				}
				
			}
			
		}
		
		public class AssetPoolEntry{
			public GameObject gameObject;
			public float lifeTime;
			
			public AssetPoolEntry(GameObject gameObject){
				this.gameObject = gameObject;
				this.lifeTime = 0.0f;
			}
			
			public override bool Equals (object obj)
			{
				AssetPoolEntry o = obj as AssetPoolEntry;
				
				if(o.gameObject == gameObject){
					return true;
				}
				
				return false;
			}
			
			public override int GetHashCode ()
			{
				return base.GetHashCode ();
			}
			
		}
		
		public static Dictionary<string,List<AssetPoolEntry>> pool = new Dictionary<string,List<AssetPoolEntry>>();
		public static bool DEBUG_MODE = false;
		
		public static GameObject Instantiate(string prefabName){
			
			string key = prefabName;
			
			if(!pool.ContainsKey(key)){
				pool.Add(key,new List<AssetPoolEntry>());
			}
			
			List<AssetPoolEntry> objectList = pool[key];
			
			if(objectList.Count > 0)
			{
			
				GameObject o = objectList[0].gameObject;
				objectList.RemoveAt(0);
				
#if UNITY_4_0
				o.SetActive(true);
#else
				o.SetActiveRecursively(true);
#endif
				
				if(DEBUG_MODE){
					Debug.Log("Instantiate: " + pool[key].Count + " assets in pool for object: " + key);
				}
				
				return o;
			
			}
			else{
				
				GameObject prefab = Resources.Load(prefabName,typeof(GameObject)) as GameObject;
				
				GameObject o = GameObject.Instantiate(prefab) as GameObject;
				o.name = key;
				
				if(DEBUG_MODE){
					Debug.Log("Instantiating " + key + " from prefab.");
				}
				
				return o;
				
			}
			
		}
		
		public static GameObject Instantiate(GameObject prefab){
			
			if(USE_RESOURCES){
				Debug.Log("Instantiating from prefab when Resource usage is enabled. This is at your discretion.");
			}
			
			string key = prefab.name;
			
			if(!pool.ContainsKey(key)){
				pool.Add(key,new List<AssetPoolEntry>());
			}
			
			List<AssetPoolEntry> objectList = pool[key];
			
			if(objectList.Count > 0)
			{
			
				GameObject o = objectList[0].gameObject;
				objectList.RemoveAt(0);
				
#if UNITY_4_0
				o.SetActive(true);
#else
				o.SetActiveRecursively(true);
#endif
				
				if(DEBUG_MODE){
					Debug.Log("Instantiate: " + pool[key].Count + " assets in pool for object: " + key);
				}
				
				return o;
			
			}
			else{
				
				GameObject o = GameObject.Instantiate(prefab) as GameObject;
				o.name = key;
				
				if(DEBUG_MODE){
					Debug.Log("Instantiating " + key + " from prefab.");
				}
				
				return o;
			
			}
		}
		
		public static void Destroy(GameObject gameObject){
			
			if(!IsPoolingEnabled()){
				
				
				if(DESTROY_IMMEDIATE){
					GameObject.DestroyImmediate(gameObject);
				}
				else{
					GameObject.Destroy(gameObject);
				}
				
				return;
			}
			
			if(assetPoolObject == null){
				assetPoolObject = new GameObject("Asset Pool");
			}
			
			string key = gameObject.name;
			
			if(!pool.ContainsKey(key)){
				pool.Add(key,new List<AssetPoolEntry>());
			}
			
			compEntry.gameObject = gameObject;
			
			if(pool[key].Contains(compEntry)){
				Debug.LogWarning("Asset Pool already contains object: " + gameObject.name + " - " + gameObject.GetInstanceID());
			}
			
			AssetPoolEntry entry = new AssetPoolEntry(gameObject);
			
			pool[key].Add(entry);
			
#if UNITY_4_0
			gameObject.SetActive(false);
#else		
			gameObject.SetActiveRecursively(false);
#endif
			
			gameObject.transform.parent = assetPoolObject.transform;
			gameObject.transform.localPosition = Vector3.zero;
			
			if(DEBUG_MODE){
				Debug.Log("Destroy:" + pool[key].Count + " assets in pool for object: " + key);
			}
		}
		
		public static void ClearPools(){
			
			if(DEBUG_MODE){
				Debug.Log("Clearing pools.");
			}
			
			foreach(string key in pool.Keys){
				
				List<AssetPoolEntry> objectList = pool[key]; 
				
				int length = objectList.Count;
				
				for(int i = 0; i < length; i++){
					if(DESTROY_IMMEDIATE){
						GameObject.DestroyImmediate(objectList[i].gameObject);
					}
					else{
						GameObject.Destroy(objectList[i].gameObject);
					}
				}
				
				objectList.Clear();
				
			}
			
			pool.Clear();
			
		}
	}
}

