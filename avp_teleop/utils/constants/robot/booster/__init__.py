ALL_LINKS = [
    "Trunk",
    "H1",
    "H2",
    "AL1",
    "AL2",
    "AL3",
    "AL4",
    "AL5",
    "AL6",
    "left_hand_link",
    "AR1",
    "AR2",
    "AR3",
    "AR4",
    "AR5",
    "AR6",
    "right_hand_link",
    "Waist",
    "Hip_Pitch_Left",
    "Hip_Roll_Left",
    "Hip_Yaw_Left",
    "Shank_Left",
    "Ankle_Cross_Left",
    "left_foot_link",
    "Hip_Pitch_Right",
    "Hip_Roll_Right",
    "Hip_Yaw_Right",
    "Shank_Right",
    "Ankle_Cross_Right",
    "right_foot_link",
]

BIMANUAL_MASK = [
    False,  # "Trunk"
    False,  # "H1"
    False,  # "H2"
    True,   # "AL1"
    True,   # "AL2"
    True,   # "AL3"
    True,   # "AL4"
    True,   # "AL5"
    True,   # "AL6"
    True,   # "left_hand_link"
    True,   # "AR1"
    True,   # "AR2"
    True,   # "AR3"
    True,   # "AR4"
    True,   # "AR5"
    True,   # "AR6"
    True,   # "right_hand_link"
    False,  # "Waist"
    False,  # "Hip_Pitch_Left"
    False,  # "Hip_Roll_Left"
    False,  # "Hip_Yaw_Left"
    False,  # "Shank_Left"
    False,  # "Ankle_Cross_Left"
    False,  # "left_foot_link"
    False,  # "Hip_Pitch_Right"
    False,  # "Hip_Roll_Right"
    False,  # "Hip_Yaw_Right"
    False,  # "Shank_Right"
    False,  # "Ankle_Cross_Right"
    False,  # "right_foot_link"
]

BASE_LINK_NAME = "Trunk"
HEAD_LINK_NAME = "H2"
L_EEF_LINK_NAME = "left_hand_link"
R_EEF_LINK_NAME = "right_hand_link"

HEAD_LINK_IDX = ALL_LINKS.index(HEAD_LINK_NAME)
L_EEF_LINK_IDX = ALL_LINKS.index(L_EEF_LINK_NAME)
R_EEF_LINK_IDX = ALL_LINKS.index(R_EEF_LINK_NAME)

DOF = 29
