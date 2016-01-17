using UnityEngine;
using UnityEditor;
using System.Collections;
using DopplerInteractive.TidyTileMapper.IconManagement;
using System.Collections.Generic;
using System.IO;
using DopplerInteractive.TidyTileMapper.Words;
using System.Reflection;
using System;

namespace DopplerInteractive.TidyTileMapper.Utility{
	
	public class TidyEditorUtility{
		
		//This class is responsible for loading prefabs, assets, etc 
		//For the Editor portion of Tidy TileMapper
		
		//Block management
		public static string blockPath = "Tidy Tile Mapper/Blocks";
		public static string coreBlockPath = "Tidy Tile Mapper/Blocks/CoreBlocks";	
		public static string nullBlockName = "EmptyBlock";
		public static string prefabPath = "Tidy Tile Mapper/Editor/Editor Prefabs";
		public static string mapChunkPrefabName = "MapChunkPrefab.prefab";
		public static string mapPath = "Tidy Tile Mapper/Maps";
		public static string meshPath = "Tidy Tile Mapper/Maps/Backgrounds";
		
		public static string functionalObjectPath = "Tidy Tile Mapper/Functional Objects";
		
		static MapChunk mapChunkPrefab;
		static Block nullBlockPrefab;
		static Block[] blockList;

		public static string iconPath = "Tidy Tile Mapper/Editor/Visual Assets/OrientationIcons";
		public static string previewPath = "Tidy Tile Mapper/Editor/Visual Assets/BlockPreviews";
			
		//Editor skinning
		public static string skinPath = "Tidy Tile Mapper/Editor/Visual Assets/UITheme/TidyTileMapper_EditorTheme.guiskin";
		static GUISkin editorSkin = null;
		
		public static Block[] GetCurrentBlocks(){
			
			if(blockList == null || blockList.Length <= 0){
				RefreshCurrentBlockList();
			}
			
			return blockList;
				
		}
				
		public static Assembly[] GetAllPlugins(){
			
			string[] plugins = Directory.GetFiles(Application.dataPath + "/" + TidyFileNames.PLUGIN_DIRECTORY);
			
			List<Assembly> assemblies = new List<Assembly>();
			
			for(int i = 0; i < plugins.Length; i++){
								
				string[] pl = plugins[i].Split('.');
				
				if(pl.Length <= 1){
					continue;
				}
				
				if(pl[pl.Length-1].ToLower() != "dll"){
					continue;
				}
				
				Assembly a = Assembly.LoadFrom(plugins[i]);
				
				if(a != null){				
					assemblies.Add(a);
				}
				
			}
			
			return assemblies.ToArray();
			
		}
		
		public static MonoScript[] GetAllFunctionalObjects(){
			
			string[] scriptNames = Directory.GetFiles(Application.dataPath + "/" + functionalObjectPath);
			
			List<MonoScript> ms = new List<MonoScript>();
			
			for(int i = 0; i < scriptNames.Length; i++){
				
				scriptNames[i] = scriptNames[i].Replace("\\","/");
				
				string[] sPath = scriptNames[i].Split('/');
				
				scriptNames[i] = sPath[sPath.Length-1];
				
				UnityEngine.Object obj = AssetDatabase.LoadAssetAtPath("Assets" + "/" + functionalObjectPath +"/"+ scriptNames[i],typeof(UnityEngine.Object));
				
				Debug.Log("Object: " + obj.name);
				
				MonoScript m = obj as MonoScript;
								
				if(m != null){
					
					System.Type t = m.GetClass();
				
					if(t != null){
						if(t.IsSubclassOf(typeof(TidyFunctionalObject))){
							ms.Add(m);
						}
					}
					else{
						Debug.Log("T is null?");
					}
				}
			}
			
			return ms.ToArray();
		}
		
		public static void RefreshCurrentBlockList(){
			
			string[] blocks = Directory.GetFiles(Application.dataPath + "/" + blockPath);
			List<Block> workingBlocks = new List<Block>();		
			
			for(int i = 0; i < blocks.Length; i++){
				
				blocks[i] = blocks[i].Replace("\\","/");
				
				string[] bPath = blocks[i].Split('/');
				
				blocks[i] = bPath[bPath.Length-1];
				
				GameObject obj = AssetDatabase.LoadAssetAtPath("Assets" + "/" + blockPath +"/"+ blocks[i],typeof(GameObject)) as GameObject;
				
				if(obj != null){
					
					OrientedBlock b = obj.GetComponent<OrientedBlock>();
					
					if(b != null){
						
						workingBlocks.Add(b);
						
						//Pre-request the preview for this block
						//To clear loads immediately
												
						//GetPreviewForGameObject(b.GetDefaultBlock(),false);
						
					}
					
				}
			}	
			
			blockList = workingBlocks.ToArray();
						
		}
		
		public static MapChunk GetMapChunkPrefab(){
			
			if(mapChunkPrefab == null){
				
				InitializeToolkitPrefabs();
				
			}
			
			return mapChunkPrefab;
			
		}
		
		public static Block GetNullBlock(){
			
			if(nullBlockPrefab == null){
				
				InitializeToolkitPrefabs();
				
			}
			
			return nullBlockPrefab;
		}
			
		static void InitializeToolkitPrefabs(){
			
			GameObject chunkObject = AssetDatabase.LoadAssetAtPath("Assets/"+prefabPath+"/"+mapChunkPrefabName,typeof(GameObject)) as GameObject;
			if(chunkObject == null){
				
				Debug.LogWarning("No MapChunk prefab found. You will not be able to create maps.");
				
			}
			else{
				
				mapChunkPrefab = chunkObject.GetComponent<MapChunk>();
				
				
				if(mapChunkPrefab == null){
					
					Debug.LogWarning("No MapChunk component found on MapChunk prefab. You will not be able to create maps.");
					
				}
			}
			
			//First, add the null block
			GameObject nullBlockObject = AssetDatabase.LoadAssetAtPath("Assets/"+coreBlockPath+"/"+nullBlockName+".prefab",typeof(GameObject)) as GameObject;
			
			if(nullBlockObject == null){
				Debug.LogWarning("No empty block found at: " +"Assets/"+coreBlockPath+"/"+nullBlockName+".prefab - This is mandatory for a healthy, happy Tidy TileMapper!");
			}
			else{
				nullBlockPrefab = nullBlockObject.GetComponent<Block>();
				
				if(nullBlockPrefab == null){
					Debug.LogWarning("No Block script found on empty block. This is required!");
				}
				
			}
			
			
		}
		
		
		
		public static GUISkin GetEditorGUISkin(){
			
			if(editorSkin == null){
			
				InitializeEditorSkin();
				
			}
						
			return editorSkin;
			
		}
		
		static void InitializeEditorSkin(){
			
			editorSkin = AssetDatabase.LoadAssetAtPath("Assets/"+skinPath,typeof(GUISkin)) as GUISkin;
					
			if(editorSkin == null){
				
				Debug.LogWarning("Editor skin is null... using default skin.");
				
			}
			
		}
		
		public static bool IsEmptyBlock(Block b){
			
			return (b.name == TidyFileNames.NULL_BLOCK_NAME);
			
		}
		
		public static Mesh SaveBackgroundMesh(Mesh m){
			AssetDatabase.CreateAsset(m,"Assets/"+meshPath+"/"+m.name+".asset");
			return AssetDatabase.LoadAssetAtPath("Assets/"+meshPath+"/"+m.name+".asset",typeof(Mesh)) as Mesh;
		}
		
		public static string SaveMapAsPrefab(GameObject map, bool overwrite){
			
			bool doesExist = DoesMapPrefabExist(map);
			
			if(!doesExist || overwrite){
				
				UnityEngine.Object o = null;
					
				if(!doesExist){
					//create it
					o = PrefabUtility.CreateEmptyPrefab("Assets/"+mapPath + "/"+ map.name + ".prefab") as UnityEngine.Object;
				}
				else{
					o = AssetDatabase.LoadAssetAtPath("Assets/"+mapPath + "/"+ map.name + ".prefab",typeof(GameObject)) as UnityEngine.Object;
				}
				
				UnityEngine.Object repObject = PrefabUtility.ReplacePrefab(map.gameObject,o) as UnityEngine.Object;
				
				AssetDatabase.Refresh();
				
				if(repObject != null){
					return TidyMessages.PUBLISH_UTILITY_SUCCESS_AT + mapPath + "/" + map.name + ".prefab";
				}
				else{
					return TidyMessages.PUBLISH_UTILITY_FAILURE;
				}
			}
			else{
			//it does exist and we're not overwriting
			int iteration = 1;
			string path = "";
			
			do{
			
				path = "Assets/"+mapPath + "/"+ map.name + "_" + iteration + ".prefab";
				iteration++;
			
			}while(DoesGameObjectExist(path));
			
			UnityEngine.Object o = PrefabUtility.CreateEmptyPrefab(path) as UnityEngine.Object;
			
			UnityEngine.Object repObject = PrefabUtility.ReplacePrefab(map.gameObject,o) as UnityEngine.Object;
							
			AssetDatabase.Refresh();
				
			if(repObject != null){
					return TidyMessages.PUBLISH_UTILITY_SUCCESS_AT + mapPath + "/" + repObject.name + ".prefab";
				}
				else{
					return TidyMessages.PUBLISH_UTILITY_FAILURE;
				}
			}
			
		}
		
		public static bool DoesMapPrefabExist(GameObject map){
			
			return DoesGameObjectExist("Assets/"+mapPath + "/"+ map.name + ".prefab");
			
			
		}
		
		static bool DoesGameObjectExist(string path){
			
			GameObject o = AssetDatabase.LoadAssetAtPath(path,typeof(GameObject)) as GameObject;
			
			return (o != null);
			
		}
		
	}
}