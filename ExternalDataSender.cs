using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using UnityEngine;
using System.Diagnostics;
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
public class ExternalDataSender : MonoBehaviour
{
    public GameObject Endoscope;
    public GameObject ExternalTracker;
    public GameObject Tracker;

    private TcpClient exClient;
    private TcpClient exClientToHolo;
    private TcpListener exClientToHoloListener;
    private NetworkStream exStream;
    private NetworkStream exStreamToHolo;

    private string serverIP = "127.0.0.1";
    Vector3 Ex_position;
    Quaternion Ex_rotation;

    Vector3 En_position;
    Quaternion En_rotation;

    private int exPort = 65431;
    private int exHoloPort = 65429;
    private Thread exThread;
    private Thread exThreadToHolo;
    private string currentTime = "0";
    Matrix4x4 TrckerFromExternalTrackerVeiw;
    Matrix4x4 EndoscopeFromExternalTrackerVeiw;
    DateTime utcNow;
    String filePath="D:\\Desktop\\ExTracker.csv";
    bool fileExists;
    private bool isApplicationRunning = true;

    // Start is called before the first frame update
    void Start()
    {
        
        StartSendThread();
        fileExists = File.Exists(filePath);

    }

    // Update is called once per frame
    void Update()
    {
        utcNow = DateTime.UtcNow;
        //currentTime = $"{(long)(utcNow - new DateTime(1970, 1, 1)).TotalMilliseconds}";
        currentTime = $"{(Stopwatch.GetTimestamp())}";
        //UnityEngine.Debug.Log($"{currentTime}");
        
        TrckerFromExternalTrackerVeiw = this.GetRelativeMatrix(Tracker.transform, ExternalTracker.transform);
        EndoscopeFromExternalTrackerVeiw = this.GetRelativeMatrix(Endoscope.transform, ExternalTracker.transform);

        Ex_position = new Vector3(TrckerFromExternalTrackerVeiw[0, 3], TrckerFromExternalTrackerVeiw[1, 3], TrckerFromExternalTrackerVeiw[2, 3]);
        Ex_rotation = TrckerFromExternalTrackerVeiw.rotation;

        En_position = new Vector3(EndoscopeFromExternalTrackerVeiw[0, 3], EndoscopeFromExternalTrackerVeiw[1, 3], EndoscopeFromExternalTrackerVeiw[2, 3]);
        En_rotation = EndoscopeFromExternalTrackerVeiw.rotation;
    }

    private void OnApplicationQuit()
    {
        isApplicationRunning = false;
        if (exClientToHoloListener != null)
        {
            exClientToHoloListener.Stop();
            exClientToHolo.Close();
        }
        exClient.Close();
        exStream.Close();
    }

    Matrix4x4 GetRelativeMatrix(Transform target, Transform reference)
    {
        //Vector3 relativePosition = reference.InverseTransformPoint(target.position);
        //Quaternion relativeRotation = Quaternion.Inverse(reference.rotation) * target.rotation;
        //Vector3 relativeScale = new Vector3(target.localScale.x / reference.localScale.x, target.localScale.y / reference.localScale.y, target.localScale.z / reference.localScale.z);
        //return Matrix4x4.TRS(relativePosition, relativeRotation, relativeScale);
        return reference.localToWorldMatrix.inverse * target.localToWorldMatrix;

    }

    void SendExData()
    {
        while(isApplicationRunning)
        {
            try
            {
                if(exClient == null || !exClient.Connected)
                {
                    if(exClient != null)
                        {
                            exClient.Close();
                        }
                    exClient = new TcpClient(serverIP, exPort);
                    exStream = exClient.GetStream();
                    UnityEngine.Debug.Log($"connected to exClient");
                }
                while (exClient != null && exStream != null && exClient.Connected)
                {
                    string dataToSend = $"{currentTime},{this.Ex_position.x},{this.Ex_position.y},{this.Ex_position.z},{this.Ex_rotation.x},{this.Ex_rotation.y},{this.Ex_rotation.z},{this.Ex_rotation.w}@";
                    byte[] data = Encoding.ASCII.GetBytes(dataToSend);
                    exStream.Write(data, 0, data.Length);


                    //using (StreamWriter writer = new StreamWriter(filePath, true)) // 'true' enables append mode
                    //{

                    //    // Write the data row
                    //    writer.WriteLine(string.Join(",", dataToSend));
                    //}

                    Console.WriteLine($"Data appended to {filePath}");
                    //UnityEngine.Debug.Log($"Ex Sent data: {dataToSend}");
                    //Thread.Sleep(Mathf.Clamp(5 * 5 + 250, 200, 300));
                    Thread.Sleep(20);
                }
            }
            catch (Exception e)
            {
                UnityEngine.Debug.Log(e);
            }
            Thread.Sleep(100);
        }

    }


    void SendExDataToHolo()
    {            
        while (isApplicationRunning)
        {
            try
            {
                if (exClientToHoloListener == null)
                {
                    exClientToHoloListener = new TcpListener(IPAddress.Any, exHoloPort);
                    exClientToHoloListener.Start();
                    UnityEngine.Debug.Log($"created exholo server");
                    

                }
                exClientToHolo = exClientToHoloListener.AcceptTcpClient();
                exStreamToHolo = exClientToHolo.GetStream();
                while(exClientToHolo != null && exClientToHolo.Connected)
                {
                    Matrix4x4 t = TrckerFromExternalTrackerVeiw;
                    Matrix4x4 e = EndoscopeFromExternalTrackerVeiw;
                    string dataToSend = $"{t.m00},{t.m01},{t.m02},{t.m03},{t.m10},{t.m11},{t.m12},{t.m13},{t.m20},{t.m21},{t.m22},{t.m23},{t.m30},{t.m31},{t.m32},{t.m33}|" +
                                        $"{e.m00},{e.m01},{e.m02},{e.m03},{e.m10},{e.m11},{e.m12},{e.m13},{e.m20},{e.m21},{e.m22},{e.m23},{e.m30},{e.m31},{e.m32},{e.m33}";
                    byte[] data = Encoding.ASCII.GetBytes(dataToSend);
                    exStreamToHolo.Write(data, 0, data.Length);
                    //UnityEngine.Debug.Log($"ExHolo Sent data: {dataToSend}");
                    exStreamToHolo.Flush();
                    //Thread.Sleep(Mathf.Clamp(5 * 5 + 250, 200, 300));
                    Thread.Sleep(10);
                }

            }
            catch (Exception e)
            {
                UnityEngine.Debug.Log($"{e}");
            }
            Thread.Sleep(100);
        }

    }

    void StartSendThread()
    {
        exThread = new Thread(SendExData);
        exThread.Start();

        exThreadToHolo = new Thread(SendExDataToHolo);
        exThreadToHolo.Start();
    }

}
