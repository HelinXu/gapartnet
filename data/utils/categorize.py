'''
Author: HelinXu xuhelin1911@gmail.com
Date: 2022-09-05 15:15:48
LastEditTime: 2022-09-16 00:29:04
Description: 
'''

BOX_1 = [
    40,
    41,
    45,
    # 52,
    55,
    58,
    60,
    63,
    64,
    # 65,
    67,
    68,
    73,
    78,
    # 86,
    89,
    # 91,
    94,
    96,
    99,
    100,
    # 101,
    102,
    117,
    122,
    124,
    128,
    # 130,
    135,
    141,
    142,
    # 148,
    149,
    153,
    164,
    165,
    167,
    169,
    174,
    178,
    184,
    186,
    # 189,
    194,
    195,
    196,
    198,
    201,
    # 202,
    204,
    206,
    208,
    209,
    210,
    # 212,
]

BUCKET_1 = [
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    8,
    9,
    10,
    11,
    12,
    13,
    15,
    16,
    20,
    22,
    23,
    26,
    27,
    30,
    32,
    35,
    36,
    38,
    39,
]

BUCKET_2 = [
    7,
    14,
    17,
    18,
    19,
    21,
    24,
    # 25,  # has 5 links
    29,
    31,
    34,
    37,
]

BUCKET_others = [
    28,
    33,
]

DRAWER_1 = [  # NOTE: 285 hasa big hole
    282,
    283,
    288,
    289,
    294,
    300,
    302,
    304,
    308,
    309,
    313,
    314,
]

DRAWER_2 = [
    274,
    277,
    281,
    287,
    291,
    296,
    307,
]

DRAWER_3 = [
    275,
]

DRAWER_4 = [
    276,
    278,
    290,
    297,
]

DRAWER_5 = [
    292,
    298,
    311,
    312,
]

DRAWER_6 = [
    301,
]

TRASHCAN_1 = [
    213,
    219,
    224,
    225,
    227,
    229,
    230,
    232,
    234,
    237,
    244,
    245,
    246,
    247,
    249,
    250,
    # 254,  # noisy mesh, bbox too big
    256,
    257,
    258,
    260,
    263,
    270,
]

CAT = {
    'box': {
        1: {
            'ids': BOX_1,
        },
    },
    'bucket': {
        1: {
            'ids': BUCKET_1,
        },
        2: {
            'ids': BUCKET_2,
        },

    },
    'drawer': {
        1: {
            'ids': DRAWER_1,
        },
        2: {
            'ids': DRAWER_2,
        },
        3: {
            'ids': DRAWER_3,
        },
        4: {
            'ids': DRAWER_4,
        },
        5: {
            'ids': DRAWER_5,
        },
        6: {
            'ids': DRAWER_6,
        },
    },
    'trashcan': {
        1: {
            'ids': TRASHCAN_1,
        }
    }
}
