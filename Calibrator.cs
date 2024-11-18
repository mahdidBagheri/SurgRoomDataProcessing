using UnityEngine;
using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEditor.PackageManager;
using System.IO;
using static UnityEditor.PlayerSettings;

public class Calibrator : MonoBehaviour
{
    public GameObject Hololens;
    public GameObject HoloTracker;
    public GameObject ExternalTracker;
    public GameObject Tracker;
    public GameObject HoloCube;
    public GameObject ExternalCube;

    public bool isRecording = false;
    private TcpClient client;
    private NetworkStream stream;
    private string serverIP = "127.0.0.1";
    private int port = 65432;
    public float captureInterval = 10;
    private float nextCaptureTimeStamp;
    private Thread captureThread;
    private bool captureDataFlag;
    private bool isTransformationRecieved = false;
    private Matrix4x4 tracking_transform;
    Matrix4x4 F;
    Matrix4x4 P;

    Vector3 Holo_position;
    Quaternion Holo_rotation;

    Vector3 Ex_position;
    Quaternion Ex_rotation;

    Matrix4x4 HoloCubeReltiveToHololes;

    void Start()
    {

        Matrix4x4 HoloTrckerFromExternalTrackerVeiw = this.GetRelativeMatrix(ExternalTracker.transform, HoloTracker.transform);
        Debug.Log($"F:\n{HoloTrckerFromExternalTrackerVeiw}");
        HoloCubeReltiveToHololes = this.GetRelativeMatrix(HoloCube.transform, HoloTracker.transform);
        
        ExternalCube.transform.localPosition = new Vector3(HoloCubeReltiveToHololes[0, 3], HoloCubeReltiveToHololes[1, 3], HoloCubeReltiveToHololes[2, 3]);
        ExternalCube.transform.localRotation = HoloCubeReltiveToHololes.rotation;

        ConnectToServer();
        StartCaptureThread();
    }

    void StartCaptureThread() 
    {
        captureDataFlag = true;
        captureThread = new Thread(SendData);
        captureThread.Start(); 
    }

void Update()
    {
        Matrix4x4 P_ = this.GetRelativeMatrix(Tracker.transform, Hololens.transform);
        Matrix4x4 F_ = this.GetRelativeMatrix(HoloTracker.transform, ExternalTracker.transform);
        Matrix4x4 TrckerFromExternalTrackerVeiw = this.GetRelativeMatrix(Tracker.transform, ExternalTracker.transform);
        Ex_position = new Vector3(TrckerFromExternalTrackerVeiw[0, 3], TrckerFromExternalTrackerVeiw[1, 3], TrckerFromExternalTrackerVeiw[2, 3]);
        Ex_rotation = TrckerFromExternalTrackerVeiw.rotation;

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
        }
    }

    private void OnValidate()
    {
        if (isRecording)
        {
            OnStart();
        }
        else
        {
            OnStop();
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

    public void OnStop()
    {
        isRecording = false;
        Debug.Log("Stopped recording data.");
        // Clean up network resources
        if (stream != null)
            stream.Close();
        if (client != null)
            client.Close();
    }

    void ConnectToServer()
    {
        try
        {
            client = new TcpClient(serverIP, port);
            stream = client.GetStream();
            Debug.Log("Connected to server");
        }
        catch (SocketException e)
        {
            Debug.LogError($"SocketException: {e}");
        }
    }

    void SendData()
    {
        for (int i = 0; i < 20; i++)
        {
            if (client != null && stream != null && client.Connected)
            {
                string dataToSend = $"{this.Holo_position.x},{this.Holo_position.y},{this.Holo_position.z},{this.Holo_rotation.x},{this.Holo_rotation.y},{this.Holo_rotation.z},{this.Holo_rotation.w}," +
                                    $"{this.Ex_position.x},{this.Ex_position.y},{this.Ex_position.z},{this.Ex_rotation.x},{this.Ex_rotation.y},{this.Ex_rotation.z},{this.Ex_rotation.w}";
                byte[] data = Encoding.ASCII.GetBytes(dataToSend);
                stream.Write(data, 0, data.Length);
                Debug.Log($"Sent data: {dataToSend}");
            }
            Thread.Sleep(250);
        }
        SendCalibrate();
        ReceiveMatrices();
    }

    void SendCalibrate()
    {
        if (client != null && stream != null && client.Connected)
        {
            string dataToSend = $"calibrate";
            byte[] data = Encoding.ASCII.GetBytes(dataToSend);
            stream.Write(data, 0, data.Length);
            Debug.Log($"Sent data: {dataToSend}");
        }
    }

    void ReceiveMatrices()
    {
        if (client != null && stream != null && client.Connected)
        {
            byte[] data = new byte[2048]; // Buffer size may vary
            int bytesRead = stream.Read(data, 0, data.Length);
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

