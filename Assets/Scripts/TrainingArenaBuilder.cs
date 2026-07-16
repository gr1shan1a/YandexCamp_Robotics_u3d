using UnityEngine;

[ExecuteAlways]
public class TrainingArenaBuilder : MonoBehaviour
{
    [Header("Robot scale calibration")]
    public Transform robotRoot;
    public float realRobotTotalLengthMeters = 0.49f;
    public bool useRobotBoundsScale = true;

    [Header("Arena real size, meters")]
    public float realArenaWidthMeters = 5.0f;
    public float realArenaLengthMeters = 6.5f;
    public float realWallHeightMeters = 0.5f;
    public float realWallThicknessMeters = 0.12f;
    public float realFloorThicknessMeters = 0.05f;

    [Header("Fallback if Robot Root is empty")]
    public float fallbackUnityUnitsPerMeter = 20f;

    [Header("Generated names")]
    public string generatedRootName = "GeneratedTrainingArena";
    public string floorName = "Floor";
    public string wallsRootName = "Walls";
    public string obstaclesRootName = "Obstacles";
    public string spawnPointsRootName = "SpawnPoints";

    [Header("Materials")]
    public Color floorColor = new Color(0.55f, 0.52f, 0.48f);
    public Color wallColor = new Color(0.38f, 0.45f, 0.52f);
    public Color obstacleColor = new Color(0.22f, 0.24f, 0.26f);
    public Color spawnColor = new Color(0.2f, 0.7f, 1.0f);

    [ContextMenu("Generate Training Arena")]
    public void GenerateTrainingArena()
    {
        float unitsPerMeter = GetUnityUnitsPerMeter();
        float width = realArenaWidthMeters * unitsPerMeter;
        float length = realArenaLengthMeters * unitsPerMeter;
        float floorThickness = realFloorThicknessMeters * unitsPerMeter;
        float wallHeight = realWallHeightMeters * unitsPerMeter;
        float wallThickness = realWallThicknessMeters * unitsPerMeter;

        Transform oldRoot = transform.Find(generatedRootName);
        if (oldRoot != null)
        {
            DestroyObject(oldRoot.gameObject);
        }

        GameObject root = CreateEmpty(generatedRootName, transform);
        GameObject wallsRoot = CreateEmpty(wallsRootName, root.transform);
        GameObject obstaclesRoot = CreateEmpty(obstaclesRootName, root.transform);
        GameObject spawnRoot = CreateEmpty(spawnPointsRootName, root.transform);

        Material floorMaterial = CreateMaterial("Arena_Floor_Material", floorColor);
        Material wallMaterial = CreateMaterial("Arena_Wall_Material", wallColor);
        Material obstacleMaterial = CreateMaterial("Arena_Obstacle_Material", obstacleColor);
        Material spawnMaterial = CreateMaterial("Arena_Spawn_Material", spawnColor);

        CreateBox(
            floorName,
            root.transform,
            new Vector3(0f, -floorThickness * 0.5f, 0f),
            new Vector3(width, floorThickness, length),
            floorMaterial
        );

        float halfWidth = width * 0.5f;
        float halfLength = length * 0.5f;
        float wallY = wallHeight * 0.5f;

        CreateBox("WallNorth", wallsRoot.transform, new Vector3(0f, wallY, halfLength), new Vector3(width, wallHeight, wallThickness), wallMaterial);
        CreateBox("WallSouth", wallsRoot.transform, new Vector3(0f, wallY, -halfLength), new Vector3(width, wallHeight, wallThickness), wallMaterial);
        CreateBox("WallEast", wallsRoot.transform, new Vector3(halfWidth, wallY, 0f), new Vector3(wallThickness, wallHeight, length), wallMaterial);
        CreateBox("WallWest", wallsRoot.transform, new Vector3(-halfWidth, wallY, 0f), new Vector3(wallThickness, wallHeight, length), wallMaterial);

        CreateObstacleMeters("Obstacle_01", obstaclesRoot.transform, -1.55f, -1.95f, 0.35f, 1.10f, 18f, unitsPerMeter, obstacleMaterial);
        CreateObstacleMeters("Obstacle_02", obstaclesRoot.transform, 1.25f, -1.25f, 1.05f, 0.30f, -28f, unitsPerMeter, obstacleMaterial);
        CreateObstacleMeters("Obstacle_03", obstaclesRoot.transform, -1.05f, 1.15f, 0.30f, 1.00f, -35f, unitsPerMeter, obstacleMaterial);
        CreateObstacleMeters("Obstacle_04", obstaclesRoot.transform, 1.25f, 1.75f, 0.90f, 0.30f, 24f, unitsPerMeter, obstacleMaterial);
        CreateObstacleMeters("Obstacle_05", obstaclesRoot.transform, 0.00f, 0.10f, 0.45f, 0.45f, 0f, unitsPerMeter, obstacleMaterial);

        CreateSpawnMarker(
            "RobotSpawn",
            spawnRoot.transform,
            new Vector3(0f, 0.03f * unitsPerMeter, -2.35f * unitsPerMeter),
            Quaternion.identity,
            spawnMaterial,
            unitsPerMeter
        );

        CreateSpawnMarker(
            "BallSpawn",
            spawnRoot.transform,
            new Vector3(0f, 0.03f * unitsPerMeter, -1.15f * unitsPerMeter),
            Quaternion.identity,
            spawnMaterial,
            unitsPerMeter
        );

        Debug.Log(
            $"Generated training arena: {realArenaWidthMeters}m x {realArenaLengthMeters}m, " +
            $"scale={unitsPerMeter:0.###} Unity units per meter, Unity size={width:0.###} x {length:0.###}."
        );
    }

    [ContextMenu("Print Robot Scale Info")]
    public void PrintRobotScaleInfo()
    {
        float unitsPerMeter = GetUnityUnitsPerMeter();
        Bounds bounds = GetRobotBounds();

        Debug.Log(
            $"Robot bounds size: {bounds.size}. " +
            $"Estimated scale: {unitsPerMeter:0.###} Unity units per meter. " +
            $"Arena Unity size: {(realArenaWidthMeters * unitsPerMeter):0.###} x {(realArenaLengthMeters * unitsPerMeter):0.###}."
        );
    }

    float GetUnityUnitsPerMeter()
    {
        if (!useRobotBoundsScale || robotRoot == null)
        {
            return Mathf.Max(0.001f, fallbackUnityUnitsPerMeter);
        }

        Bounds bounds = GetRobotBounds();
        float robotUnityLength = Mathf.Max(bounds.size.x, bounds.size.z);
        if (robotUnityLength <= 0.001f || realRobotTotalLengthMeters <= 0.001f)
        {
            return Mathf.Max(0.001f, fallbackUnityUnitsPerMeter);
        }

        return robotUnityLength / realRobotTotalLengthMeters;
    }

    Bounds GetRobotBounds()
    {
        if (robotRoot == null)
        {
            return new Bounds(Vector3.zero, Vector3.zero);
        }

        Renderer[] renderers = robotRoot.GetComponentsInChildren<Renderer>();
        Bounds bounds = new Bounds(robotRoot.position, Vector3.zero);
        bool hasBounds = false;

        foreach (Renderer renderer in renderers)
        {
            if (!hasBounds)
            {
                bounds = renderer.bounds;
                hasBounds = true;
            }
            else
            {
                bounds.Encapsulate(renderer.bounds);
            }
        }

        return bounds;
    }

    GameObject CreateEmpty(string objectName, Transform parent)
    {
        GameObject obj = new GameObject(objectName);
        obj.transform.SetParent(parent);
        obj.transform.localPosition = Vector3.zero;
        obj.transform.localRotation = Quaternion.identity;
        obj.transform.localScale = Vector3.one;
        return obj;
    }

    GameObject CreateBox(string objectName, Transform parent, Vector3 localPosition, Vector3 localScale, Material material)
    {
        GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obj.name = objectName;
        obj.transform.SetParent(parent);
        obj.transform.localPosition = localPosition;
        obj.transform.localRotation = Quaternion.identity;
        obj.transform.localScale = localScale;

        Collider collider = obj.GetComponent<Collider>();
        if (collider != null)
        {
            collider.isTrigger = false;
        }

        Renderer renderer = obj.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.sharedMaterial = material;
        }

        return obj;
    }

    void CreateObstacleMeters(
        string objectName,
        Transform parent,
        float xMeters,
        float zMeters,
        float widthMeters,
        float lengthMeters,
        float yRotation,
        float unitsPerMeter,
        Material material
    )
    {
        float obstacleHeight = 0.35f * unitsPerMeter;
        GameObject obj = CreateBox(
            objectName,
            parent,
            new Vector3(xMeters * unitsPerMeter, obstacleHeight * 0.5f, zMeters * unitsPerMeter),
            new Vector3(widthMeters * unitsPerMeter, obstacleHeight, lengthMeters * unitsPerMeter),
            material
        );

        obj.transform.localRotation = Quaternion.Euler(0f, yRotation, 0f);
    }

    void CreateSpawnMarker(
        string objectName,
        Transform parent,
        Vector3 localPosition,
        Quaternion localRotation,
        Material material,
        float unitsPerMeter
    )
    {
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        marker.name = objectName;
        marker.transform.SetParent(parent);
        marker.transform.localPosition = localPosition;
        marker.transform.localRotation = localRotation;
        marker.transform.localScale = new Vector3(0.25f * unitsPerMeter, 0.02f * unitsPerMeter, 0.25f * unitsPerMeter);

        Collider collider = marker.GetComponent<Collider>();
        if (collider != null)
        {
            collider.isTrigger = true;
        }

        Renderer renderer = marker.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.sharedMaterial = material;
        }
    }

    Material CreateMaterial(string materialName, Color color)
    {
        Shader shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null)
        {
            shader = Shader.Find("Standard");
        }

        Material material = new Material(shader);
        material.name = materialName;
        material.color = color;
        return material;
    }

    void DestroyObject(GameObject obj)
    {
        if (Application.isPlaying)
        {
            Destroy(obj);
        }
        else
        {
            DestroyImmediate(obj);
        }
    }
}
