using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;
using System.Reflection;

[ExecuteInEditMode]
[CustomEditor(typeof(MonoBehaviour))]
public class FixMissingScripts : Editor
{
	public static bool tried;
	public static GameObject tryThisObject;
	
	public override void OnInspectorGUI ()
	{
		
		if(target.GetType()!=typeof(MonoBehaviour) && target.GetType() != typeof(UnityEngine.Object))
		{
			base.OnInspectorGUI();
			return;
		}
		EditorPrefs.SetBool("Fix", GUILayout.Toggle(EditorPrefs.GetBool("Fix", true), "Fix broken scripts"));
		if(!EditorPrefs.GetBool("Fix", true))
		{
			GUILayout.Label("*** SCRIPT MISSING ***");
			return;
		}
		Initialize();
		var iterator = this.serializedObject.GetIterator();
		var first = true;
		while(iterator.NextVisible(first))
		{
			first = false;
			if(iterator.name == "m_Script" && iterator.objectReferenceValue == null)
			{
				if(tryThisObject == (target as Component).gameObject)
					tried = true;
				var script = iterator.Copy();
				var candidates = scripts.ToList();
				while(iterator.NextVisible(false) && candidates.Count>0)
				{
					candidates = candidates.Where(c=>c.properties.ContainsKey(iterator.name)).ToList();
				}
				if(candidates.Count==1)
				{
					script.objectReferenceValue = candidates[0].script;
	
					serializedObject.ApplyModifiedProperties();
					serializedObject.UpdateIfDirtyOrScript();
				}
				else if(candidates.Count > 0)
				{
					foreach(var candidate in candidates)
					{
						if(GUILayout.Button("Use " + candidate.script.name))
						{
							script.objectReferenceValue = candidate.script;
	
							serializedObject.ApplyModifiedProperties();
							serializedObject.UpdateIfDirtyOrScript();
						}
					}
				}
				else
				{
					GUILayout.Label("> No suitable scripts were found");
				}
				break;
			}
		}
		base.OnInspectorGUI ();
		
	}
	
	class ScannedScript
	{
		public Dictionary<string, FieldInfo> properties;
		public int id;
		public MonoScript script;
	}
	static List<ScannedScript> scripts;
	static bool _initialized = false;
	bool _localInit;
    void Initialize()
	{
		if(!_localInit)
		{
			EditorApplication.projectWindowChanged += ()=>{
				Repaint();
			};
			_localInit = true;
		}
		if(_initialized)
			return;
		_initialized = true;
		
		ScanAll();
		
	}
	
	void ScanAll()
	{
		scripts = Resources.FindObjectsOfTypeAll(typeof(MonoScript))
			.Cast<MonoScript>()
			.Where(c=>c.hideFlags == 0)
			.Where(c=>c.GetClass() != null)
			.Select(c=>new ScannedScript { id = c.GetInstanceID(), 
				script = c,
				properties = c.GetClass().GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
					.Where(p=>p.IsPublic || (!p.IsPublic && p.IsDefined(typeof(SerializeField), false)))
					.ToDictionary(p=>p.Name)
			})
			.ToList();
	}
	
}
