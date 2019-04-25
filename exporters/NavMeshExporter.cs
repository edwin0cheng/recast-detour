using UnityEngine;
using UnityEngine.AI;
using System.Collections;
using UnityEditor;
using System.IO;
using System.Text;

namespace Editor
{

    // Reference documents
    // https://cloud.tencent.com/developer/article/1006053

    public class NavMeshExporterWindow : EditorWindow
    {
        [MenuItem("Fourp/NavMesh Exporter")]
        public static void ShowWindow()
        {
            var window = EditorWindow.GetWindow<NavMeshExporterWindow>();
            var rt = window.position;

            rt.height = 100;
            window.position = rt;
            rt.center = new Rect(0f, 0f, Screen.currentResolution.width, Screen.currentResolution.height).center;
            window.position = rt;
        }

        int currentPickerWindow;
        string lastStatus;

        void ExportNavMesh(out string error)
        {
            var tgn = NavMesh.CalculateTriangulation();
            if (tgn.areas.Length == 0)
            {
                error = "No navmesh was found.";
                lastStatus = null;
                return;
            }

            // We only use the first settings
            var settings = NavMesh.GetSettingsByID(0);

            var path = EditorUtility.SaveFilePanel("Save NavMesh as Obj", "", "navmesh" + ".obj", "obj");
            if (string.IsNullOrEmpty(path))
            {
                error = null;
                return;
            }

            MeshToObjFile(tgn, settings, path);

            var builder = new StringBuilder();
            builder.AppendFormat("last export: {0} \n", System.DateTime.Now);
            builder.AppendFormat("areas count: {0} \n", tgn.areas.Length);
            builder.AppendFormat("indices count: {0} \n", tgn.indices.Length);
            builder.AppendFormat("vertices count: {0} \n", tgn.vertices.Length);
            builder.AppendFormat("path: {0} \n", tgn.areas.Length);

            lastStatus = builder.ToString();
            error = null;
        }

        void OnGUI()
        {
            currentPickerWindow = EditorGUIUtility.GetControlID(FocusType.Passive) + 100;

            GUILayout.Label("Export", EditorStyles.boldLabel);

            if (GUILayout.Button("Export To Obj"))
            {
                string error;
                ExportNavMesh(out error);
                if (!string.IsNullOrEmpty(error))
                {
                    EditorUtility.DisplayDialog("Error", error, "Ok");
                }
            }

            if (!string.IsNullOrEmpty(lastStatus))
            {
                EditorGUILayout.HelpBox(lastStatus, MessageType.Info);
            }
        }

        static string MeshToObjString(NavMeshTriangulation tgn, NavMeshBuildSettings settings)
        {
            StringBuilder sb = new StringBuilder();

            /*
                /// Walkable height in nav mesh in World Unit
                walkable_height: f32,
                /// Walkable Radius in nav mesh in World Unit
                walkable_radius: f32,
                /// Walkable climb height in World Unit
                walkable_climb: f32,

                /// Cell size in world unit
                cell_size: f32,
                /// Cell height in world unit
                cell_height: f32,            
            */

            // add comment for augmented data
            sb.Append("# walkable_height ").Append(settings.agentHeight).Append("\n");
            sb.Append("# walkable_radius ").Append(settings.agentRadius).Append("\n");
            sb.Append("# walkable_climb ").Append(settings.agentClimb).Append("\n");
            
            sb.Append("# cell_size ").Append(settings.voxelSize).Append("\n");
            sb.Append("# cell_height ").Append(settings.voxelSize).Append("\n");

            sb.Append("\n");

            sb.Append("g ").Append("navmesh").Append("\n");
            foreach (Vector3 v in tgn.vertices)
            {
                sb.Append(string.Format("v {0} {1} {2}\n", v.x, v.y, v.z));
            }

            for (var i = 0; i < tgn.indices.Length; i += 3)
            {
                // Note: obj format is 1-based.
                sb.Append(string.Format("f {0} {1} {2}\n", tgn.indices[i]+1, tgn.indices[i+1]+1, tgn.indices[i+2]+1 ));
            }

            return sb.ToString();
        }

        static void MeshToObjFile(NavMeshTriangulation tgn, NavMeshBuildSettings settings, string filename)
        {
            using (StreamWriter sw = new StreamWriter(filename))
            {
                sw.Write(MeshToObjString(tgn, settings));
            }
        }
    }
}