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
using System.Diagnostics;
using UnityEngine.InputSystem;
using System.Collections;
using Unity.PlasticSCM.Editor.UI;

public class Calibrator : MonoBehaviour
{
    public GameObject Hololens;
    public GameObject HoloTracker;
    //public GameObject ExternalTracker;
    public GameObject Tracker;
    public GameObject HoloCube;
    public GameObject ExternalCube;
    public GameObject HoloEndoscope;

    public bool isRecording = false;
    private TcpClient holoClient;
    private NetworkStream holoStream;

    //private TcpClient recClient;
    private TcpClient exrecClient;
    
    //private NetworkStream recStream;
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

    private bool applicationIsRunning = true;

    public bool shouldUpdate = true;

    bool isConnectedToExHolo = false;
    Matrix4x4 F;
    Matrix4x4 P;

    Vector3 Holo_position;
    Quaternion Holo_rotation;


    Matrix4x4 HoloCubeReltiveToHololes = Matrix4x4.identity;
    private string currentTime = "0";

    Matrix4x4 TrckerFromExternalTrackerVeiw = Matrix4x4.identity;
    Matrix4x4 EndoscopeFromExternalTrackerVeiw = Matrix4x4.identity;

    DateTime utcNow;
    String filePath="D:\\Desktop\\Holo.csv";
    bool fileExists;

    TcpListener recListener = null ;

    private Vector3 RotationOffset = Vector3.zero;
    void Start()
    {
        
        //Matrix4x4 HoloTrckerFromExternalTrackerVeiw = this.GetRelativeMatrix(ExternalTracker.transform, HoloTracker.transform);
        //Debug.Log($"F:\n{HoloTrckerFromExternalTrackerVeiw}");
        //HoloCubeReltiveToHololes = this.GetRelativeMatrix(HoloCube.transform, Hololens.transform);
        HoloCubeReltiveToHololes = Matrix4x4.identity;
        HoloCubeReltiveToHololes[0,3] = 0.0f;
        HoloCubeReltiveToHololes[1,3] = 0.0f;
        HoloCubeReltiveToHololes[2,3] = 0.5f;
        ExternalCube.transform.localPosition = new Vector3(HoloCubeReltiveToHololes[0, 3], HoloCubeReltiveToHololes[1, 3], HoloCubeReltiveToHololes[2, 3]);
        ExternalCube.transform.localRotation = HoloCubeReltiveToHololes.rotation;

        UnityEngine.Debug.Log("before connect to server");
        //ConnectToRecServer();
        UnityEngine.Debug.Log("connected to server");
        StartSendThread();
        UnityEngine.Debug.Log("StartSendThread");
        fileExists = File.Exists(filePath);

    }

    public static Matrix4x4 ApplyRotationOffset(Matrix4x4 originalMatrix, Vector3 rotationOffset)
    {
        // Create a rotation matrix from the quaternion
        Matrix4x4 rotationMatrix = Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(rotationOffset), Vector3.one);

        // Apply the rotation to the original matrix
        return rotationMatrix * originalMatrix;
    }



    void StartSendThread() 
    {
        captureDataFlag = true;
        holoThread = new Thread(SendHoloData);
        holoThread.Start();


        recThread = new Thread(ReceiveMatrices);
        recThread.Start();

        exrecThread = new Thread(ReceiveExMatrice);
        exrecThread.Start();
        isConnectedToExHolo = true;

    }

    private void OnApplicationQuit()
    {
        applicationIsRunning = false;
        if (recListener != null)
        {
            recListener.Stop();
        }
        if (holoStream != null)
        {
            holoStream.Close();
        }

        //recStream.Close();
        //exrecStream.Close();
        //exrecThread.Join();
        //recThread.Join();
        //holoThread.Join();
    }

    void Update()
    {
        utcNow = DateTime.UtcNow;
        currentTime = $"{(long)(utcNow - new DateTime(1970, 1, 1)).TotalMilliseconds}";
        //TrckerFromExternalTrackerVeiw = this.GetRelativeMatrix(Tracker.transform, ExternalTracker.transform);

        // Debug.Log(currentTime);
        //Matrix4x4 P_ = this.GetRelativeMatrix(Tracker.transform, Hololens.transform);
        //Matrix4x4 F_ = this.GetRelativeMatrix(HoloTracker.transform, ExternalTracker.transform);

        //Matrix4x4 hololensFromHololoTrackerVeiw = this.GetRelativeMatrix(Hololens.transform, HoloTracker.transform);
        Matrix4x4 hololensFromHololoTrackerVeiw = Hololens.transform.localToWorldMatrix;
        Holo_position = new Vector3(hololensFromHololoTrackerVeiw[0, 3], hololensFromHololoTrackerVeiw[1, 3], hololensFromHololoTrackerVeiw[2, 3]);
        Holo_rotation = hololensFromHololoTrackerVeiw.rotation;
        UnityEngine.Debug.Log(isTransformationRecieved);
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

            var calcEndoscope = (ApplyRotationOffset(F,RotationOffset) * EndoscopeFromExternalTrackerVeiw);
            HoloEndoscope.transform.rotation = calcEndoscope.rotation;
            HoloEndoscope.transform.position = new Vector3(calcEndoscope[0,3], calcEndoscope[1, 3], calcEndoscope[2, 3]);



        }
        float ROTATION_RESOLUTION = 0.05f;
        if (Keyboard.current.numpad1Key.isPressed) RotationOffset.x -= ROTATION_RESOLUTION;
        else if (Keyboard.current.numpad2Key.isPressed) RotationOffset.y -= ROTATION_RESOLUTION;
        else if (Keyboard.current.numpad3Key.isPressed) RotationOffset.z -= ROTATION_RESOLUTION;
        else if (Keyboard.current.numpad4Key.isPressed) RotationOffset.x = 0;
        else if (Keyboard.current.numpad5Key.isPressed) RotationOffset.y = 0;
        else if (Keyboard.current.numpad6Key.isPressed) RotationOffset.z = 0;
        else if (Keyboard.current.numpad7Key.isPressed) RotationOffset.x += ROTATION_RESOLUTION;
        else if (Keyboard.current.numpad8Key.isPressed) RotationOffset.y += ROTATION_RESOLUTION;
        else if (Keyboard.current.numpad9Key.isPressed) RotationOffset.z += ROTATION_RESOLUTION;
        else if (Keyboard.current.numpad0Key.isPressed) shouldUpdate = !shouldUpdate;


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
        UnityEngine.Debug.Log("Started recording data.");
    }

    void SendHoloData()
    {
        while (applicationIsRunning)
        {                    

            try {
                if (holoClient == null || !holoClient.Connected)
                {
                    if(holoClient != null)
                    {
                        holoClient.Close();
                    }
                    holoClient = new TcpClient(serverIP, holoPort);
                    holoStream = holoClient.GetStream();
                }
                while (holoClient != null && holoStream != null && holoClient.Connected && applicationIsRunning)
                {
                    string dataToSend = $"{this.currentTime},{this.Holo_position.x},{this.Holo_position.y},{this.Holo_position.z},{this.Holo_rotation.x},{this.Holo_rotation.y},{this.Holo_rotation.z},{this.Holo_rotation.w}@";
                    byte[] data = Encoding.ASCII.GetBytes(dataToSend);
                    holoStream.Write(data, 0, data.Length);
                    //holoStream.Flush();
                    using (StreamWriter writer = new StreamWriter(filePath, true)) // 'true' enables append mode
                    {
                        writer.WriteLine(string.Join(",", dataToSend));
                    }
                    //Console.WriteLine($"Data appended to {filePath}");
                    Thread.Sleep(10);
                }
            }
            catch (Exception e) {
                UnityEngine.Debug.LogError($"SocketException: {e}");
            }
            Thread.Sleep(100);
        }
    }

    void ReceiveExMatrice()
    {
        while (applicationIsRunning)
        {
            try
            {
                if (exrecClient == null)
                {
                    exrecClient = new TcpClient(serverIP, exrecPort);
                    exrecStream = exrecClient.GetStream();
                    UnityEngine.Debug.Log("Connected to exrecserver");
                }

                while (exrecClient != null && exrecStream != null && exrecClient.Connected && applicationIsRunning)
                {
                    byte[] data = new byte[2048]; // Buffer size may vary
                    int bytesRead = exrecStream.Read(data, 0, data.Length);
                    string receivedData = Encoding.ASCII.GetString(data, 0, bytesRead);
                    if (receivedData == "")
                    {
                        exrecClient.Close();
                        exrecClient = null;
                        break;
                    }
                    UnityEngine.Debug.Log($"{receivedData}");
                    string[] splittedData = receivedData.Split('|');
                    if (splittedData.Length == 2)
                    {
                        TrckerFromExternalTrackerVeiw = ParseMatrix(splittedData[0]);
                        EndoscopeFromExternalTrackerVeiw = ParseMatrix(splittedData[1]);
                        UnityEngine.Debug.Log($"{EndoscopeFromExternalTrackerVeiw}");
                    }
                    else
                    {
                        UnityEngine.Debug.LogError($"Invalid data received: {receivedData}");
                    }

                }
            } 
            catch(Exception e)
            {
                UnityEngine.Debug.Log($"{e}");
            }
            Thread.Sleep(100);
        }
    }




    string RecieveMatricesFromClient()
    {
         UnityEngine.Debug.Log("Started");
        try
        {
            UnityEngine.Debug.Log(recListener == null);
            if (recListener == null)
            {
                recListener = new TcpListener(IPAddress.Any, recPort);                
                recListener.Start();
                UnityEngine.Debug.Log("Connected to exholo started");
            }

            TcpClient recClient = recListener.AcceptTcpClient();
            NetworkStream recStream = recClient.GetStream();
            UnityEngine.Debug.Log("Connected to exholo server");
            byte[] buffer = new byte[1024];
            int bytesRead = recStream.Read(buffer, 0, buffer.Length);
            string receivedMessage = Encoding.ASCII.GetString(buffer, 0, bytesRead);
            UnityEngine.Debug.Log("Received message: " + receivedMessage);
            //recStream.Close();
            recClient.Close();
            return receivedMessage;
            return "";

        }
        catch (SocketException e)
        {
            UnityEngine.Debug.LogError($"SocketException: {e}");
            return "";
        }
    }

    void ReceiveMatrices()
    {
        while (applicationIsRunning)
        {
            try
            {
                string receivedData = RecieveMatricesFromClient();

                string[] matricesData = receivedData.Split('|'); // Assuming matrices are separated by '|'

                if (matricesData.Length == 2 && shouldUpdate)
                {
                    F = ParseMatrix(matricesData[0]);
                    P = ParseMatrix(matricesData[1]);

                    UnityEngine.Debug.Log("F:\n" + F);
                    UnityEngine.Debug.Log("P:\n" + P);
                    isTransformationRecieved= true;
                }
                else
                {
                    UnityEngine.Debug.LogError("Invalid data received");
                }
            } catch (Exception e)
            {
                UnityEngine.Debug.LogError($"connection error {e}");
                Thread.Sleep(50);
            }
        }
    }

    Matrix4x4 ParseMatrix(string data)
    {
        string[] values = data.Split(',');
        if (values.Length != 16)
        {
            UnityEngine.Debug.LogError("Invalid matrix data");
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

