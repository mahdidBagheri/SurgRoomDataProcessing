using UnityEngine;
using System.Collections.Generic;
using System.IO;
using System.Text;

public class ManipulatorAndRecorder : MonoBehaviour
{
    public GameObject Hololens;
    public GameObject HoloTracker;
    public GameObject Tracker;
    public GameObject ExternalTracker;

    public float dt = 1.0f;

    private float myTimer;
    private int myIndex;
    private System.Random random;
    private string holoDataFilePath;
    private string trackerDataFilePath;
    private string dataFolderPath;

    void Start()
    {
        myTimer = 0.0f;
        myIndex = 0;
        random = new System.Random();

        // Create Data folder if it doesn't exist
        dataFolderPath = Path.Combine(Application.dataPath, "Data");
        if (!Directory.Exists(dataFolderPath))
        {
            Directory.CreateDirectory(dataFolderPath);
        }

        // Create CSV files with incremented names in the Data folder
        holoDataFilePath = CreateCSV("HoloData");
        trackerDataFilePath = CreateCSV("Data");
    }

    void Update()
    {
        myTimer += Time.deltaTime;

        //Mathf.PerlinNoise(myTimer)

        if (myTimer >= dt)
        {
            // Randomly manipulate the Hololens object
            //Hololens.transform.position = new Vector3(
            //    random.NextFloat(-10.0f, 10.0f),
            //    random.NextFloat(-10.0f, 10.0f),
            //    random.NextFloat(-10.0f, 10.0f)
            //);
            //Hololens.transform.rotation = Random.rotation;

            // Save the relative transformation of Hololens to HoloTracker
            Vector3 hololensToHoloTrackerPosition = HoloTracker.transform.InverseTransformPoint(Hololens.transform.position);
            Quaternion hololensToHoloTrackerRotation = Quaternion.Inverse(HoloTracker.transform.rotation) * Hololens.transform.rotation;
            AppendToCSV(holoDataFilePath, myIndex, Time.time, hololensToHoloTrackerPosition, hololensToHoloTrackerRotation);

            // Save the relative transformation of Tracker to ExternalTracker
            Vector3 trackerToExternalTrackerPosition = ExternalTracker.transform.InverseTransformPoint(Tracker.transform.position);
            Quaternion trackerToExternalTrackerRotation = Quaternion.Inverse(ExternalTracker.transform.rotation) * Tracker.transform.rotation;
            AppendToCSV(trackerDataFilePath, myIndex, Time.time, trackerToExternalTrackerPosition, trackerToExternalTrackerRotation);

            myTimer = 0.0f;
            myIndex++;
        }
    }

    string CreateCSV(string baseFileName)
    {
        int fileIndex = 0;
        string filePath;

        // Find the next available file index
        do
        {
            fileIndex++;
            filePath = Path.Combine(dataFolderPath, $"{baseFileName}{fileIndex}.csv");
        } while (File.Exists(filePath));

        // Create the new file and add headers
        //StringBuilder sb = new StringBuilder();
        //sb.AppendLine("Index,Timestamp,Tx,Ty,Tz,q0,qx,qy,qz");
        //File.WriteAllText(filePath, sb.ToString());

        return filePath;
    }

    void AppendToCSV(string filePath, int myIndex, float currTimeStamp, Vector3 position, Quaternion rotation)
    {
        StringBuilder sb = new StringBuilder();
        sb.AppendLine($"{position.x},{position.y},{position.z},{rotation.w},{rotation.x},{rotation.y},{rotation.z}");
        File.AppendAllText(filePath, sb.ToString());
    }
}

public static class RandomExtensions
{
    public static float NextFloat(this System.Random random, float minValue, float maxValue)
    {
        return (float)(random.NextDouble() * (maxValue - minValue) + minValue);
    }
}
