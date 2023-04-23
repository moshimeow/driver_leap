#include "stdafx.h"

#include "Devices/CLeapController/CLeapControllerIndex.h"

#include "Devices/CLeapController/CControllerButton.h"
#include "Core/CDriverConfig.h"
#include "Utils/CGestureMatcher.h"
#include "Utils/Utils.h"

#include "bones.h"


#include "xrt/xrt_defines.h"
#include "math/m_mathinclude.h"
#include "math/m_api.h"

extern const glm::mat4 g_identityMatrix;
extern const vr::VRBoneTransform_t g_openHandGesture[];
extern const glm::vec4 g_zeroPoint;

enum HandFinger : size_t
{
    HF_Thumb = 0U,
    HF_Index,
    HF_Middle,
    HF_Ring,
    HF_Pinky,

    HF_Count
};

CLeapControllerIndex::CLeapControllerIndex(unsigned char p_hand)
{
    m_hand = (p_hand % CH_Count);
    m_type = CT_IndexKnuckle;
    m_serialNumber.assign((m_hand == CH_Left) ? "LHR-E217CD00" : "LHR-E217CD01");

    for(size_t i = 0U; i < HSB_Count; i++) m_boneTransform[i] = g_openHandGesture[i];
    m_skeletonHandle = vr::k_ulInvalidInputComponentHandle;

    if(m_hand == CH_Right)
    {
        // Transformation inversion along 0YZ plane
        for(size_t i = 1U; i < HSB_Count; i++)
        {
            m_boneTransform[i].position.v[0] *= -1.f;

            switch(i)
            {
                case HSB_Wrist:
                {
                    m_boneTransform[i].orientation.y *= -1.f;
                    m_boneTransform[i].orientation.z *= -1.f;
                } break;

                case HSB_Thumb0:
                case HSB_IndexFinger0:
                case HSB_MiddleFinger0:
                case HSB_RingFinger0:
                case HSB_PinkyFinger0:
                {
                    m_boneTransform[i].orientation.z *= -1.f;
                    std::swap(m_boneTransform[i].orientation.x, m_boneTransform[i].orientation.w);
                    m_boneTransform[i].orientation.w *= -1.f;
                    std::swap(m_boneTransform[i].orientation.y, m_boneTransform[i].orientation.z);
                } break;
            }
        }
    }

}

CLeapControllerIndex::~CLeapControllerIndex()
{
}

void CLeapControllerIndex::ChangeBoneOrientation(glm::quat &p_rot)
{
    std::swap(p_rot.x, p_rot.z);
    p_rot.z *= -1.f;
    if(m_hand == CH_Left)
    {
        p_rot.x *= -1.f;
        p_rot.y *= -1.f;
    }
}

void CLeapControllerIndex::ChangeAuxTransformation(glm::vec3 &p_pos, glm::quat &p_rot)
{
    p_pos.y *= -1.f;
    p_pos.z *= -1.f;

    std::swap(p_rot.x, p_rot.w);
    p_rot.w *= -1.f;
    std::swap(p_rot.y, p_rot.z);
    p_rot.y *= -1.f;
}

size_t CLeapControllerIndex::GetFingerBoneIndex(size_t p_id)
{
    size_t l_result = 0U;
    switch(p_id)
    {
        case HF_Thumb:
            l_result = HSB_Thumb0;
            break;
        case HF_Index:
            l_result = HSB_IndexFinger0;
            break;
        case HF_Middle:
            l_result = HSB_MiddleFinger0;
            break;
        case HF_Ring:
            l_result = HSB_RingFinger0;
            break;
        case HF_Pinky:
            l_result = HSB_PinkyFinger0;
            break;
    }
    return l_result;
}

void CLeapControllerIndex::ActivateInternal()
{
    // Properties
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_TrackingSystemName_String, "lighthouse");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_SerialNumber_String, m_serialNumber.c_str());
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_WillDriftInYaw_Bool, false);
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceIsWireless_Bool, true);
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceIsCharging_Bool, false);
    vr::VRProperties()->SetFloatProperty(m_propertyContainer, vr::Prop_DeviceBatteryPercentage_Float, 1.f); // Always charged

    vr::HmdMatrix34_t l_matrix = { -1.f, 0.f, 0.f, 0.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f };
    vr::VRProperties()->SetProperty(m_propertyContainer, vr::Prop_StatusDisplayTransform_Matrix34, &l_matrix, sizeof(vr::HmdMatrix34_t), vr::k_unHmdMatrix34PropertyTag);

    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_Firmware_UpdateAvailable_Bool, false);
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_Firmware_ManualUpdate_Bool, false);
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_Firmware_ManualUpdateURL_String, "https://developer.valvesoftware.com/wiki/SteamVR/HowTo_Update_Firmware");
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_DeviceCanPowerOff_Bool, true);
    vr::VRProperties()->SetInt32Property(m_propertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_Firmware_ForceUpdateRequired_Bool, false);
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_Identifiable_Bool, true);
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_Firmware_RemindUpdate_Bool, false);
    vr::VRProperties()->SetInt32Property(m_propertyContainer, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_TrackPad);
    vr::VRProperties()->SetInt32Property(m_propertyContainer, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger);
    vr::VRProperties()->SetInt32Property(m_propertyContainer, vr::Prop_ControllerRoleHint_Int32, (m_hand == CH_Left) ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand);
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_HasDisplayComponent_Bool, false);
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_HasCameraComponent_Bool, false);
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool, false);
    vr::VRProperties()->SetBoolProperty(m_propertyContainer, vr::Prop_HasVirtualDisplayComponent_Bool, false);
    vr::VRProperties()->SetInt32Property(m_propertyContainer, vr::Prop_ControllerHandSelectionPriority_Int32, 0);
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ModelNumber_String, (m_hand == CH_Left) ? "Knuckles Left" : "Knuckles Right");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_RenderModelName_String, (m_hand == CH_Left) ? "{indexcontroller}valve_controller_knu_1_0_left" : "{indexcontroller}valve_controller_knu_1_0_right");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ManufacturerName_String, "Valve");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_TrackingFirmwareVersion_String, "1562916277 watchman@ValveBuilder02 2019-07-12 FPGA 538(2.26/10/2) BL 0 VRC 1562916277 Radio 1562882729");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_HardwareRevision_String, "product 17 rev 14.1.9 lot 2019/4/20 0");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ConnectedWirelessDongle_String, "C2F75F5986-DIY"); // Changed
    vr::VRProperties()->SetUint64Property(m_propertyContainer, vr::Prop_HardwareRevision_Uint64, 286130441U);
    vr::VRProperties()->SetUint64Property(m_propertyContainer, vr::Prop_FirmwareVersion_Uint64, 1562916277U);
    vr::VRProperties()->SetUint64Property(m_propertyContainer, vr::Prop_FPGAVersion_Uint64, 538U);
    vr::VRProperties()->SetUint64Property(m_propertyContainer, vr::Prop_VRCVersion_Uint64, 1562916277U);
    vr::VRProperties()->SetUint64Property(m_propertyContainer, vr::Prop_RadioVersion_Uint64, 1562882729U);
    vr::VRProperties()->SetUint64Property(m_propertyContainer, vr::Prop_DongleVersion_Uint64, 1558748372U);
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_Firmware_ProgrammingTarget_String, (m_hand == CH_Left) ? "LHR-E217CD00" : "LHR-E217CD01"); // Changed
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ResourceRoot_String, "indexcontroller");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_RegisteredDeviceType_String, (m_hand == CH_Left) ? "valve/index_controllerLHR-E217CD00" : "valve/index_controllerLHR-E217CD01"); // Changed
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_InputProfilePath_String, "{indexcontroller}/input/index_controller_profile.json");
    vr::VRProperties()->SetInt32Property(m_propertyContainer, vr::Prop_Axis2Type_Int32, vr::k_eControllerAxis_Trigger);
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_NamedIconPathDeviceOff_String, (m_hand == CH_Left) ? "{indexcontroller}/icons/left_controller_status_off.png" : "{indexcontroller}/icons/right_controller_status_off.png");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, (m_hand == CH_Left) ? "{indexcontroller}/icons/left_controller_status_searching.gif" : "{indexcontroller}/icons/right_controller_status_searching.gif");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, (m_hand == CH_Left) ? "{indexcontroller}/icons/left_controller_status_searching_alert.gif" : "{indexcontroller}/icons//right_controller_status_searching_alert.gif");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_NamedIconPathDeviceReady_String, (m_hand == CH_Left) ? "{indexcontroller}/icons/left_controller_status_ready.png" : "{indexcontroller}/icons//right_controller_status_ready.png");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, (m_hand == CH_Left) ? "{indexcontroller}/icons/left_controller_status_ready_alert.png" : "{indexcontroller}/icons//right_controller_status_ready_alert.png");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, (m_hand == CH_Left) ? "{indexcontroller}/icons/left_controller_status_error.png" : "{indexcontroller}/icons//right_controller_status_error.png");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, (m_hand == CH_Left) ? "{indexcontroller}/icons/left_controller_status_off.png" : "{indexcontroller}/icons//right_controller_status_off.png");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, (m_hand == CH_Left) ? "{indexcontroller}/icons/left_controller_status_ready_low.png" : "{indexcontroller}/icons//right_controller_status_ready_low.png");
    vr::VRProperties()->SetStringProperty(m_propertyContainer, vr::Prop_ControllerType_String, "knuckles");

    // Input
    if(m_buttons.empty())
    {
        for(size_t i = 0U; i < IB_Count; i++) m_buttons.push_back(new CControllerButton());
    }

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/system/click", &m_buttons[IB_SystemClick]->GetHandleRef());
    m_buttons[IB_SystemClick]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/system/touch", &m_buttons[IB_SystemTouch]->GetHandleRef());
    m_buttons[IB_SystemTouch]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/trigger/click", &m_buttons[IB_TriggerClick]->GetHandleRef());
    m_buttons[IB_TriggerClick]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/trigger/touch", &m_buttons[IB_TriggerTouch]->GetHandleRef());
    m_buttons[IB_TriggerClick]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/trigger/value", &m_buttons[IB_TriggerValue]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    m_buttons[IB_TriggerValue]->SetInputType(CControllerButton::IT_Float);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/trackpad/x", &m_buttons[IB_TrackpadX]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    m_buttons[IB_TrackpadX]->SetInputType(CControllerButton::IT_Float);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/trackpad/y", &m_buttons[IB_TrackpadY]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    m_buttons[IB_TrackpadY]->SetInputType(CControllerButton::IT_Float);

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/trackpad/touch", &m_buttons[IB_TrackpadTouch]->GetHandleRef());
    m_buttons[IB_TrackpadTouch]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/trackpad/force", &m_buttons[IB_TrackpadForce]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    m_buttons[IB_TrackpadForce]->SetInputType(CControllerButton::IT_Float);

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/grip/touch", &m_buttons[IB_GripTouch]->GetHandleRef());
    m_buttons[IB_GripTouch]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/grip/force", &m_buttons[IB_GripForce]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    m_buttons[IB_GripForce]->SetInputType(CControllerButton::IT_Float);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/grip/value", &m_buttons[IB_GripValue]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    m_buttons[IB_GripValue]->SetInputType(CControllerButton::IT_Float);

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/thumbstick/click", &m_buttons[IB_ThumbstickClick]->GetHandleRef());
    m_buttons[IB_ThumbstickClick]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/thumbstick/touch", &m_buttons[IB_ThumbstickTouch]->GetHandleRef());
    m_buttons[IB_ThumbstickTouch]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/thumbstick/x", &m_buttons[IB_ThumbstickX]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    m_buttons[IB_ThumbstickX]->SetInputType(CControllerButton::IT_Float);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/thumbstick/y", &m_buttons[IB_ThumbstickY]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    m_buttons[IB_ThumbstickY]->SetInputType(CControllerButton::IT_Float);

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/a/click", &m_buttons[IB_AClick]->GetHandleRef());
    m_buttons[IB_AClick]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/a/touch", &m_buttons[IB_ATouch]->GetHandleRef());
    m_buttons[IB_ATouch]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/b/click", &m_buttons[IB_BClick]->GetHandleRef());
    m_buttons[IB_BClick]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateBooleanComponent(m_propertyContainer, "/input/b/touch", &m_buttons[IB_BTouch]->GetHandleRef());
    m_buttons[IB_BTouch]->SetInputType(CControllerButton::IT_Boolean);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/finger/index", &m_buttons[IB_FingerIndex]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    m_buttons[IB_FingerIndex]->SetInputType(CControllerButton::IT_Float);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/finger/middle", &m_buttons[IB_FingerMiddle]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    m_buttons[IB_FingerMiddle]->SetInputType(CControllerButton::IT_Float);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/finger/ring", &m_buttons[IB_FingerRing]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    m_buttons[IB_FingerRing]->SetInputType(CControllerButton::IT_Float);

    vr::VRDriverInput()->CreateScalarComponent(m_propertyContainer, "/input/finger/pinky", &m_buttons[IB_FingerPinky]->GetHandleRef(), vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    m_buttons[IB_FingerPinky]->SetInputType(CControllerButton::IT_Float);

    const vr::EVRSkeletalTrackingLevel l_trackingLevel = ((CDriverConfig::GetTrackingLevel() == CDriverConfig::TL_Partial) ? vr::VRSkeletalTracking_Partial : vr::VRSkeletalTracking_Full);
    switch(m_hand)
    {
        case CH_Left:
            vr::VRDriverInput()->CreateSkeletonComponent(m_propertyContainer, "/input/skeleton/left", "/skeleton/hand/left", "/pose/raw", l_trackingLevel, nullptr, 0U, &m_skeletonHandle);
            break;
        case CH_Right:
            vr::VRDriverInput()->CreateSkeletonComponent(m_propertyContainer, "/input/skeleton/right", "/skeleton/hand/right", "/pose/raw", l_trackingLevel, nullptr, 0U, &m_skeletonHandle);
            break;
    }

    vr::VRDriverInput()->CreateHapticComponent(m_propertyContainer, "/output/haptic", &m_haptic);
}

struct xrt_quat leap_oxr_rot = {0};
enum xrt_space_relation_flags valid_flags =
	(enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
									XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
									XRT_SPACE_RELATION_POSITION_VALID_BIT |
									XRT_SPACE_RELATION_POSITION_TRACKED_BIT);


void ulv4_leap_convert_joint_xrt(LEAP_VECTOR position, LEAP_QUATERNION rotation, float width, uint32_t joint_index, struct xrt_hand_joint_set *out_set)
{
	struct xrt_hand_joint_value *joint = &out_set->values.hand_joint_set_default[joint_index];
	struct xrt_pose *pose = &joint->relation.pose;
	pose->position.x = -position.x / 1000.0f;
	pose->position.y = -position.z / 1000.0f;
	pose->position.z = -position.y / 1000.0f;
	pose->orientation.w =  rotation.w;
	pose->orientation.x = -rotation.x;
	pose->orientation.y = -rotation.z;
	pose->orientation.z = -rotation.y;

	math_quat_rotate(&pose->orientation, &leap_oxr_rot, &pose->orientation);

	joint->radius = width / 1000 / 2; // Leap is in mm, OpenXR in m

	joint->relation.relation_flags = valid_flags;
}

void ulv4_leap_convert_digit_xrt(LEAP_DIGIT digit, uint32_t metacarpal_joint, struct xrt_hand_joint_set *out_set)
{
	uint8_t digit_joint_start = 0;
	if(metacarpal_joint == XRT_HAND_JOINT_THUMB_METACARPAL)
		digit_joint_start = 1; // In Leap v4, the thumb is missing the metacarpal joint

	// Convert all bones except for the tip
	LEAP_BONE bone = digit.bones[0];
	for(uint8_t j=digit_joint_start; j<4; ++j) {
		bone = digit.bones[j];
		ulv4_leap_convert_joint_xrt(bone.prev_joint, bone.rotation, bone.width, metacarpal_joint+j-digit_joint_start, out_set);
	}

	// Then convert the tip
	ulv4_leap_convert_joint_xrt(bone.next_joint, bone.rotation, bone.width, metacarpal_joint+4-digit_joint_start, out_set);
}

void ulv4_leap_convert_hand_xrt(const LEAP_HAND* hand, struct xrt_hand_joint_set *out_set)
{
	// Convert the wrist (wrist is treated as the arm bone closest to the hand aka farthest from heart)
	ulv4_leap_convert_joint_xrt(hand->arm.next_joint, hand->middle.metacarpal.rotation, hand->arm.width, XRT_HAND_JOINT_WRIST, out_set);

	// Convert the palm
	const LEAP_BONE *middle_metacarpal = &hand->middle.metacarpal;
	LEAP_VECTOR palm_position = {0};

	palm_position.x = (middle_metacarpal->prev_joint.x + middle_metacarpal->next_joint.x) / 2;
	palm_position.y = (middle_metacarpal->prev_joint.y + middle_metacarpal->next_joint.y) / 2;
	palm_position.z = (middle_metacarpal->prev_joint.z + middle_metacarpal->next_joint.z) / 2;

	ulv4_leap_convert_joint_xrt(palm_position, middle_metacarpal->rotation, hand->palm.width, XRT_HAND_JOINT_PALM, out_set);

	// Convert all the digits
	ulv4_leap_convert_digit_xrt(hand->pinky,  XRT_HAND_JOINT_LITTLE_METACARPAL, out_set);
	ulv4_leap_convert_digit_xrt(hand->ring,   XRT_HAND_JOINT_RING_METACARPAL,   out_set);
	ulv4_leap_convert_digit_xrt(hand->middle, XRT_HAND_JOINT_MIDDLE_METACARPAL, out_set);
	ulv4_leap_convert_digit_xrt(hand->index,  XRT_HAND_JOINT_INDEX_METACARPAL,  out_set);
	ulv4_leap_convert_digit_xrt(hand->thumb,  XRT_HAND_JOINT_THUMB_METACARPAL,  out_set);
}


void
ulv4_create_leap_rot() {
	struct xrt_vec3 forward_axis = {0, 0, 1};
	struct xrt_vec3 right_axis = {1, 0, 0};

	struct xrt_quat rot1 = {0};
	math_quat_from_angle_vector(M_PI, &forward_axis, &rot1);

	struct xrt_quat rot2 = {0};
	math_quat_from_angle_vector(-M_PI/2, &right_axis, &rot2);

	math_quat_rotate(&rot1, &rot2, &leap_oxr_rot);
}


void CLeapControllerIndex::UpdateGestures(const LEAP_HAND *p_hand)
{
    if(p_hand)
    {
        std::vector<float> l_gestures;
        CGestureMatcher::GetGestures(p_hand, l_gestures);

        m_buttons[IB_TriggerValue]->SetValue(l_gestures[CGestureMatcher::HG_Trigger]);
        m_buttons[IB_TriggerTouch]->SetState(l_gestures[CGestureMatcher::HG_Trigger] >= 0.5f);
        m_buttons[IB_TriggerClick]->SetState(l_gestures[CGestureMatcher::HG_Trigger] >= 0.75f);

        m_buttons[IB_GripValue]->SetValue(l_gestures[CGestureMatcher::HG_Grab]);
        m_buttons[IB_GripTouch]->SetState(l_gestures[CGestureMatcher::HG_Grab] >= 0.75f);
        m_buttons[IB_GripForce]->SetValue((l_gestures[CGestureMatcher::HG_Grab] >= 0.9f) ? (l_gestures[CGestureMatcher::HG_Grab] - 0.9f) * 10.f : 0.f);
        
        m_buttons[IB_FingerIndex]->SetValue(l_gestures[CGestureMatcher::HG_IndexBend]);
        m_buttons[IB_FingerMiddle]->SetValue(l_gestures[CGestureMatcher::HG_MiddleBend]);
        m_buttons[IB_FingerRing]->SetValue(l_gestures[CGestureMatcher::HG_RingBend]);
        m_buttons[IB_FingerPinky]->SetValue(l_gestures[CGestureMatcher::HG_PinkyBend]);

        ulv4_create_leap_rot();
        struct xrt_hand_joint_set blah = {};
        ulv4_leap_convert_hand_xrt(p_hand, &blah);

        // struct xrt_pose root_pose = {};

        // if (m_hand == CH_Right) {
        //     root_pose.position = {0.034038f, 0.036503f, 0.164722f};
        //     root_pose.orientation.w = -0.055147f;
        //     root_pose.orientation.x = -0.078608f;
        //     root_pose.orientation.y = 0.920279f;
        //     root_pose.orientation.z = -0.379296f;

        //     math_quat_invert(&root_pose.orientation, &root_pose.orientation);

        // } else {
        //     root_pose.position = {-0.034038f, 0.036503f, 0.164722f};
        //     root_pose.orientation.w = -0.055147f;
        //     root_pose.orientation.x = -0.078608f;
        //     root_pose.orientation.y = -0.920279f;
        //     root_pose.orientation.z = 0.379296f;
        //     math_quat_invert(&root_pose.orientation, &root_pose.orientation);
        // }



        HandJointSetToBoneTransform(blah,
                                 m_boneTransform,
                                 (m_hand == CH_Left) ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand,
                                 blah.values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation.pose);
        if (m_hand == CH_Right) {
            m_boneTransform[0] = rightOpenPose[0];
            m_boneTransform[1] = rightOpenPose[1];
        } else {
            m_boneTransform[0] = leftOpenPose[0];
            m_boneTransform[1] = leftOpenPose[1];
        }

        // for(size_t i = 0U; i < 5U; i++)
        // {
        //     const LEAP_DIGIT &l_finger = p_hand->digits[i];
        //     size_t l_transformIndex = GetFingerBoneIndex(i);
        //     if(l_transformIndex != HSB_Thumb0) l_transformIndex++;

        //     LEAP_QUATERNION l_leapRotation = p_hand->palm.orientation;
        //     glm::quat l_segmentRotation;
        //     ConvertQuaternion(l_leapRotation, l_segmentRotation);

        //     for(size_t j = 1U; j < 4U; j++)
        //     {
        //         const glm::quat l_prevSegmentRotationInv = glm::inverse(l_segmentRotation);
        //         l_leapRotation = l_finger.bones[j].rotation;
        //         ConvertQuaternion(l_leapRotation, l_segmentRotation);

        //         glm::quat l_segmentResult = l_prevSegmentRotationInv * l_segmentRotation;
        //         ChangeBoneOrientation(l_segmentResult);
        //         if(l_transformIndex == HSB_Thumb0)
        //         {
        //             std::swap(l_segmentResult.z, l_segmentResult.w);
        //             l_segmentResult.w *= -1.f;
        //             if(m_hand == CH_Right)
        //             {
        //                 std::swap(l_segmentResult.x, l_segmentResult.w);
        //                 l_segmentResult.w *= -1.f;
        //                 std::swap(l_segmentResult.y, l_segmentResult.z);
        //                 l_segmentResult.y *= -1.f;
        //             }
        //         }
        //         ConvertQuaternion(l_segmentResult, m_boneTransform[l_transformIndex].orientation);
        //         l_transformIndex++;
        //     }
        // }

        // // Update aux bones
        // glm::vec3 l_position;
        // glm::quat l_rotation;
        // ConvertVector3(m_boneTransform[HSB_Wrist].position, l_position);
        // ConvertQuaternion(m_boneTransform[HSB_Wrist].orientation, l_rotation);
        // const glm::mat4 l_wristMat = glm::translate(g_identityMatrix, l_position) * glm::mat4_cast(l_rotation);

        // for(size_t i = HF_Thumb; i < HF_Count; i++)
        // {
        //     glm::mat4 l_chainMat(l_wristMat);
        //     const size_t l_chainIndex = GetFingerBoneIndex(i);
        //     for(size_t j = 0U; j < ((i == HF_Thumb) ? 3U : 4U); j++)
        //     {
        //         ConvertVector3(m_boneTransform[l_chainIndex + j].position, l_position);
        //         ConvertQuaternion(m_boneTransform[l_chainIndex + j].orientation, l_rotation);
        //         l_chainMat = l_chainMat * (glm::translate(g_identityMatrix, l_position)*glm::mat4_cast(l_rotation));
        //     }
        //     l_position = l_chainMat * g_zeroPoint;
        //     l_rotation = glm::quat_cast(l_chainMat);
        //     if(m_hand == CH_Left) ChangeAuxTransformation(l_position, l_rotation);
        //     ConvertVector3(l_position, m_boneTransform[HSB_Aux_Thumb + i].position);
        //     ConvertQuaternion(l_rotation, m_boneTransform[HSB_Aux_Thumb + i].orientation);
        // }
    }
}

void CLeapControllerIndex::UpdateInputInternal()
{
    vr::VRDriverInput()->UpdateSkeletonComponent(m_skeletonHandle, vr::VRSkeletalMotionRange_WithController, m_boneTransform, HSB_Count);
    vr::VRDriverInput()->UpdateSkeletonComponent(m_skeletonHandle, vr::VRSkeletalMotionRange_WithoutController, m_boneTransform, HSB_Count);
}

void CLeapControllerIndex::SetButtonState(size_t p_button, bool p_state)
{
    if(p_button < m_buttons.size())
        m_buttons[p_button]->SetState(p_state);
}

void CLeapControllerIndex::SetButtonValue(size_t p_button, float p_value)
{
    if(p_button < m_buttons.size())
        m_buttons[p_button]->SetValue(p_value);
}
