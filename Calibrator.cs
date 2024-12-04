using UnityEngine;
using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEditor.PackageManager;
using System.IO;
using static UnityEditor.PlayerSettings;
using System.Net;

public class Calibrator : MonoBehaviour
{
    public GameObject Hololens;
    public GameObject HoloTracker;
    public GameObject ExternalTracker;
    public GameObject Tracker;
    public GameObject HoloCube;
    public GameObject ExternalCube;
    public GameObject HoloEndoscope;

    public bool isRecording = false;
    private TcpClient holoClient;
    private NetworkStream holoStream;

    private TcpClient recClient;
    private TcpClient exrecClient;
    private NetworkStream recStream;
    private NetworkStream exrecStream;
    private string serverIP = "127.0.0.1";
    private int holoPort = 65432;

    private int recPort = 65430;
    private int exrecPort = 65429;
    public float captureInterval = 10;
    private float nextCaptureTimeStamp;
    private Thread holoThread;

    private Thread recThread;
    private Thread exrecThread;
    private bool captureDataFlag;
    private bool isTransformationRecieved = false;
    private Matrix4x4 tracking_transform;

    bool isConnectedToExHolo = false;
    Matrix4x4 F;
    Matrix4x4 P;

    Vector3 Holo_position;
    Quaternion Holo_rotation;


    Matrix4x4 HoloCubeReltiveToHololes;
    private string currentTime = "0";

    Matrix4x4 TrckerFromExternalTrackerVeiw = Matrix4x4.identity;
    Matrix4x4 EndoscopeFromExternalTrackerVeiw = Matrix4x4.identity;

    void Start()
    {

        //Matrix4x4 HoloTrckerFromExternalTrackerVeiw = this.GetRelativeMatrix(ExternalTracker.transform, HoloTracker.transform);
        //Debug.Log($"F:\n{HoloTrckerFromExternalTrackerVeiw}");
        HoloCubeReltiveToHololes = this.GetRelativeMatrix(HoloCube.transform, HoloTracker.transform);
        
        ExternalCube.transform.localPosition = new Vector3(HoloCubeReltiveToHololes[0, 3], HoloCubeReltiveToHololes[1, 3], HoloCubeReltiveToHololes[2, 3]);
        ExternalCube.transform.localRotation = HoloCubeReltiveToHololes.rotation;

        ConnectToHoloServer();
        ConnectToRecServer();
        
        StartSendThread();
    }

    void StartSendThread() 
    {
        captureDataFlag = true;
        holoThread = new Thread(SendHoloData);
        holoThread.Start();


        recThread = new Thread(ReceiveMatrices);
        recThread.Start();


    }

    private void OnApplicationQuit()
    {
        holoStream.Close();
        recStream.Close();
        exrecStream.Close();
    }

    void Update()
    {
        if (Time.time > 1.0 && !isConnectedToExHolo)
        {
            ConnectToExRecServer();
            exrecThread = new Thread(ReceiveExMatrice);
            exrecThread.Start();
            isConnectedToExHolo = true;
        }
        //TrckerFromExternalTrackerVeiw = this.GetRelativeMatrix(Tracker.transform, ExternalTracker.transform);

        currentTime = Time.time.ToString();
        //Debug.Log(currentTime);
        //Matrix4x4 P_ = this.GetRelativeMatrix(Tracker.transform, Hololens.transform);
        //Matrix4x4 F_ = this.GetRelativeMatrix(HoloTracker.transform, ExternalTracker.transform);

        Matrix4x4 hololensFromHololoTrackerVeiw = this.GetRelativeMatrix(Hololens.transform, HoloTracker.transform);
        Holo_position = new Vector3(hololensFromHololoTrackerVeiw[0, 3], hololensFromHololoTrackerVeiw[1, 3], hololensFromHololoTrackerVeiw[2, 3]);
        Holo_rotation = hololensFromHololoTrackerVeiw.rotation;

        if (isTransformationRecieved)
        {
            tracking_transform = F * TrckerFromExternalTrackerVeiw * P.inverse;
            Matrix4x4 ExCubeNew = (tracking_transform) * HoloCubeReltiveToHololes;
            ExternalCube.transform.rotation = ExCubeNew.rotation;
            ExternalCube.transform.position = new Vector3(ExCubeNew.m03, ExCubeNew.m13, ExCubeNew.m23);
        
            Matrix4x4 HoloCubeNew = (hololensFromHololoTrackerVeiw) * HoloCubeReltiveToHololes;
            HoloCube.transform.rotation = HoloCubeNew.rotation;
            HoloCube.transform.position = new Vector3(HoloCubeNew.m03, HoloCubeNew.m13, HoloCubeNew.m23);
            int a = 0;

            var calcEndoscope = (F * EndoscopeFromExternalTrackerVeiw);
            HoloEndoscope.transform.rotation = calcEndoscope.rotation;
            HoloEndoscope.transform.position = new Vector3(calcEndoscope[0,3], calcEndoscope[1, 3], calcEndoscope[2, 3]);
        }
    }

    Matrix4x4 GetRelativeMatrix(Transform target, Transform reference)
    {
        Vector3 relativePosition = reference.InverseTransformPoint(target.position);
        Quaternion relativeRotation = Quaternion.Inverse(reference.rotation) * target.rotation;
        Vector3 relativeScale = new Vector3(target.localScale.x / reference.localScale.x, target.localScale.y / reference.localScale.y, target.localScale.z / reference.localScale.z);

        return Matrix4x4.TRS(relativePosition, relativeRotation, relativeScale);
    }

    public void OnStart()
    {
        isRecording = true;
        Debug.Log("Started recording data.");
    }


    void ConnectToHoloServer()
    {
        try
        {
            holoClient = new TcpClient(serverIP, holoPort);
            holoStream = holoClient.GetStream();
            Debug.Log("Connected to server");
        }
        catch (SocketException e)
        {
            Debug.LogError($"SocketException: {e}");
        }
    }

    void ConnectToRecServer()
    {
        try
        {
            recClient = new TcpClient(serverIP, recPort);
            recStream = recClient.GetStream();
            Debug.Log("Connected to server");
        }
        catch (SocketException e)
        {
            Debug.LogError($"SocketException: {e}");
        }
    }

    void ConnectToExRecServer()
    {
        try
        {
            exrecClient = new TcpClient(serverIP, exrecPort);
            exrecStream = exrecClient.GetStream();
            Debug.Log("Connected to server");
        }
        catch (SocketException e)
        {
            Debug.LogError($"SocketException: {e}");
        }
    }

    void SendHoloData()
    {

        while (holoClient != null && holoStream != null && holoClient.Connected)
        {
            string dataToSend = $"{this.currentTime},{this.Holo_position.x},{this.Holo_position.y},{this.Holo_position.z},{this.Holo_rotation.x},{this.Holo_rotation.y},{this.Holo_rotation.z},{this.Holo_rotation.w}";
            byte[] data = Encoding.ASCII.GetBytes(dataToSend);
            holoStream.Write(data, 0, data.Length);
            //Debug.Log($"Holo Sent data: {dataToSend}");
            //Thread.Sleep(Mathf.Clamp(5 * 5 + 250, 200, 300));
            Thread.Sleep(50);

        }
    }

    void ReceiveExMatrice()
    {
        while (exrecClient != null && exrecStream != null && exrecClient.Connected)
        {
            byte[] data = new byte[2048]; // Buffer size may vary
            int bytesRead = exrecStream.Read(data, 0, data.Length);
            string[] receivedData = Encoding.ASCII.GetString(data, 0, bytesRead).Split('|');
            if (receivedData.Length == 2)
            {
                TrckerFromExternalTrackerVeiw = ParseMatrix(receivedData[0]);
                EndoscopeFromExternalTrackerVeiw = ParseMatrix(receivedData[1]);
                //Debug.Log($"{EndoscopeFromExternalTrackerVeiw}");
            }
            else
            {
                Debug.LogError("Invalid data received");
            }

        }
    }


    void ReceiveMatrices()
    {
        while (recClient != null && recStream != null && recClient.Connected)
        {
            byte[] data = new byte[2048]; // Buffer size may vary
            int bytesRead = recStream.Read(data, 0, data.Length);
            string receivedData = Encoding.ASCII.GetString(data, 0, bytesRead);

            string[] matricesData = receivedData.Split('|'); // Assuming matrices are separated by '|'

            if (matricesData.Length == 2)
            {
                F = ParseMatrix(matricesData[0]);
                P = ParseMatrix(matricesData[1]);

                Debug.Log("F:\n" + F);
                Debug.Log("P:\n" + P);
                isTransformationRecieved= true;
            }
            else
            {
                Debug.LogError("Invalid data received");
            }
        }
    }

    Matrix4x4 ParseMatrix(string data)
    {
        string[] values = data.Split(',');
        if (values.Length != 16)
        {
            Debug.LogError("Invalid matrix data");
            return Matrix4x4.identity;
        }
        Matrix4x4 matrix = new Matrix4x4();
        for (int i = 0; i < 16; i++)
        {
            matrix[i] = float.Parse(values[i]);
        }
        return matrix.transpose;
    }

}

