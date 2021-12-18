﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Valve.VR;

namespace leap_control
{
    class VRManager
    {
        bool m_initialized = false;

        readonly Core m_core = null;

        CVRSystem m_vrSystem;
        VREvent_t m_vrEvent;
        static uint ms_eventSize = 0;
        bool m_active = false;

        ulong m_notificationOverlay = 0;
        uint m_leapDevice = 0;
        uint m_leftHandController = 0;
        uint m_rightHandController = 0;

        TrackedDevicePose_t[] m_trackedPoses = null;

        public VRManager(Core f_core)
        {
            m_core = f_core;

            m_trackedPoses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
        }

        public bool Initialize()
        {
            if(!m_initialized)
            {
                EVRInitError l_initError = EVRInitError.None;
                m_vrSystem = OpenVR.Init(ref l_initError, EVRApplicationType.VRApplication_Overlay);
                if(l_initError == EVRInitError.None)
                {
                    OpenVR.Overlay.CreateOverlay("leap.control.notification", "Ultraleap Control", ref m_notificationOverlay);

                    // Find fake Leap Motion station device
                    for(uint i = 0; i < OpenVR.k_unMaxTrackedDeviceCount; i++)
                    {
                        ETrackedPropertyError l_propertyError = ETrackedPropertyError.TrackedProp_Success;
                        ulong l_property = m_vrSystem.GetUint64TrackedDeviceProperty(i, ETrackedDeviceProperty.Prop_VendorSpecific_Reserved_Start, ref l_propertyError);
                        if((l_propertyError == ETrackedPropertyError.TrackedProp_Success) && (l_property == 0x4C4D6F74696F6E))
                        {
                            m_leapDevice = i;
                            break;
                        }
                    }

                    ms_eventSize = (uint)System.Runtime.InteropServices.Marshal.SizeOf(m_vrEvent);
                    m_initialized = true;
                    m_active = true;
                }

            }

            return m_initialized;
        }

        public void Terminate()
        {
            if(m_initialized)
            {

                m_initialized = false;
            }
        }

        public bool DoPulse()
        {
            m_vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseRawAndUncalibrated, 0f, m_trackedPoses);

            while(m_vrSystem.PollNextEvent(ref m_vrEvent, ms_eventSize))
            {
                switch(m_vrEvent.eventType)
                {
                    case (uint)EVREventType.VREvent_Quit:
                        m_active = false;
                        break;
                    case (uint)EVREventType.VREvent_TrackedDeviceDeactivated:
                    {
                        if(m_leftHandController == m_vrEvent.trackedDeviceIndex)
                            m_leftHandController = 0;
                        if(m_rightHandController == m_vrEvent.trackedDeviceIndex)
                            m_rightHandController = 0;
                    }
                    break;
                }
            }

            if(m_trackedPoses[OpenVR.k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
            {
                GlmSharp.mat4 l_matrix = GlmSharp.mat4.Identity;
                m_trackedPoses[OpenVR.k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking.Convert(ref l_matrix);
                m_core.ControlManager.SetHeadTransform(l_matrix);
            }

            if(m_leftHandController != 0)
            {
                GlmSharp.mat4 l_matrix = GlmSharp.mat4.Identity;
                m_trackedPoses[m_leftHandController].mDeviceToAbsoluteTracking.Convert(ref l_matrix);
                m_core.ControlManager.SetHandTransform(ControlManager.Hand.Left, l_matrix);
            }
            else
                m_leftHandController = m_vrSystem.GetTrackedDeviceIndexForControllerRole(ETrackedControllerRole.LeftHand);

            if(m_rightHandController != 0)
            {
                GlmSharp.mat4 l_matrix = GlmSharp.mat4.Identity;
                m_trackedPoses[m_rightHandController].mDeviceToAbsoluteTracking.Convert(ref l_matrix);
                m_core.ControlManager.SetHandTransform(ControlManager.Hand.Right, l_matrix);
            }
            else
                m_rightHandController = m_vrSystem.GetTrackedDeviceIndexForControllerRole(ETrackedControllerRole.RightHand);

            return m_active;
        }

        public void SendMessage(string f_message)
        {
            Console.WriteLine(f_message);
            if(m_leapDevice != 0)
            {
                StringBuilder l_stringBuilder = new StringBuilder(32);
                OpenVR.Debug.DriverDebugRequest(m_leapDevice, f_message, l_stringBuilder, 32);
            }
        }

        public void ShowNotification(string f_message)
        {
            // Why is it borked? It worked in C++, ffs
            uint l_notification = 0;
            NotificationBitmap_t l_bitmap = new NotificationBitmap_t();
            l_bitmap.m_pImageData = (IntPtr)0;
            l_bitmap.m_nHeight = 0;
            l_bitmap.m_nWidth = 0;
            l_bitmap.m_nBytesPerPixel = 0;
            Console.WriteLine(OpenVR.Notifications.CreateNotification(m_notificationOverlay, 500, EVRNotificationType.Transient, f_message, EVRNotificationStyle.None, ref l_bitmap, ref l_notification).ToString());
        }
    }
}