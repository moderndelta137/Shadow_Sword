using UnityEngine;
using UnityEditor;
using System;
using System.Collections.Generic;
using System.IO;

//A very specific data class
//That will keep a dictionary of Textures corresponding to names
//It may be populated from a source directory

namespace DopplerInteractive.TidyTileMapper.IconManagement{
	
	public class TextureDictionary{
		
		protected Dictionary<string,Texture2D> textureDictionary;
		protected string sourceDirectory = "";
		
		//sourceDirectory is a string representing the location of the textures, within the Unity project
		//excluding the "Assets" prefix
		public TextureDictionary(string sourceDirectory){
			
			textureDictionary = new Dictionary<string, Texture2D>();
			this.sourceDirectory = sourceDirectory;
			
			PopulateDictionary(sourceDirectory);
		
		}
		
		public Texture2D GetTexture(string name){
		
			if(textureDictionary.ContainsKey(name)){
				return textureDictionary[name];
			}
			
			return null;
		}
		
		public void LoadTextureForName(string objectName){
		
			Texture2D texture = AssetDatabase.LoadAssetAtPath("Assets/"+sourceDirectory+"/"+objectName+".png",typeof(Texture)) as Texture2D;
			
			if(texture != null){
				
				if(textureDictionary.ContainsKey(objectName)){
										
					textureDictionary[objectName] = texture;
				}
				else{
					
					textureDictionary.Add(objectName,texture);
				}
			}
		}
		
		public override string ToString ()
		{
			string textureDictionaryString = "Source: " + sourceDirectory;
			
			foreach(string key in textureDictionary.Keys){
				textureDictionaryString += "["+key +","+textureDictionary[key]+"],";
			}
			
			return textureDictionaryString;
		}
		
		void PopulateDictionary(string sourceDirectory){
			
			string absoluteDirectoryPath = Application.dataPath + "/"+ sourceDirectory;
			
			if(Directory.Exists(absoluteDirectoryPath)){
			
				string[] allFiles = Directory.GetFiles(absoluteDirectoryPath);
				
				//For each file in our directory, strip the filename out
				for(int i = 0; i < allFiles.Length; i++){
					
					allFiles[i] = allFiles[i].Replace("\\","/");
					
					string[] pathSplit = allFiles[i].Split('/');
					
					string fileName = pathSplit[pathSplit.Length-1];
										
					string filePath = "Assets/"+sourceDirectory+"/"+fileName;
					
					Texture2D texture = AssetDatabase.LoadAssetAtPath(filePath,typeof(Texture2D)) as Texture2D;
				
					if(texture != null){
						
						string[] nameSplit = fileName.Split('.');
						
						string name = nameSplit[0];
						
						textureDictionary.Add(name,texture);
						
					}
				
				}
				
			}
			else{
				Debug.LogWarning("Directory does not exist: " + absoluteDirectoryPath);
			}
		}
	}
}