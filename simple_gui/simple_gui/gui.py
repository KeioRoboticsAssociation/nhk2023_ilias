import PySimpleGUI as sg
from ament_index_python.packages import get_package_share_directory
from rogilink2_interfaces.msg import Ping
from rclpy.node import Node

img_folder_path = get_package_share_directory('simple_gui') + '/img'
rr_hard_id = (0, "FR_S", "FR", "FL_S", "FL", "RR_S", "RR", "RL_S", "RL", 0, 0,
              0, 0, "limit_sw", "steer_sensor", "sensor", "LED")
discon = ('White', 'Black')
connected = ('Black', 'White')


def create_state_frame_button(name: str, key: str):
    return sg.Button(name, key=key, size=(10, 2), font='Helvetica 20')


def create_menu_frame_button(name: str, key: str):
    return sg.Button(name, key=key, size=(4, 2), font='Helvetica 20')


# GUI class
class GUI:

    sg.theme('Topanga')
    ################### Header ###################
    header = [[
        sg.Button('Home', font='Helvetica 10', size=(10, 2)),
        sg.Text('Ilias2023',
                font='Helvetica 30',
                size=(20, 2),
                justification='center'),
        sg.Button(
            key='emergency_stop',
            image_filename=f'{img_folder_path}/emergency_stop.png',
            image_size=(100, 100),
            button_color=(sg.theme_background_color(),
                          sg.theme_background_color()),
        )
    ]]
    home = [
        [
            sg.Button('RR', font='Helvetica 20', size=(10, 2)),
            sg.Button('ER', font='Helvetica 20', size=(10, 2))
        ],
    ]

    ################### ERHOME menu ###################
    er_state_frame = sg.Frame(
        'State',
        [
            # text box to choose state
            [
                sg.Text('State', size=(5, 1), font='Helcetica 20'),
                sg.InputCombo(('Start', 'Restart', 'PickupLeft', 'PickupRight',
                               'PreShot', 'Shot', 'End'),
                              key='er_state_select',
                              size=(15, 1),
                              font='Helcetica 20'),
                sg.Button('Set',
                          key='er_set_state',
                          size=(5, 1),
                          font='Helcetica 20')
            ],
            [
                sg.Column([[
                    create_state_frame_button('Start', 'er_start'),
                    create_state_frame_button('Restart', 'er_restart')
                ]],
                          justification='center'),
            ],
            [
                sg.Column([[
                    create_state_frame_button('Idle', 'er_idle'),
                    create_state_frame_button('Manual', 'er_manual')
                ]],
                          justification='center')
            ],
            [
                sg.Column([[
                    create_state_frame_button('Forward', 'er_forward'),
                ]],
                          justification='center')
            ]
        ],
        size=(500, 800))
    er_menu_frame = sg.Frame('Menu', [[]], size=(500, 800))

    erhome = [
        [
            sg.Text('ER',
                    font='Helvetica 20',
                    size=(10, 2),
                    justification='center')
        ],
        [er_state_frame, er_menu_frame],
    ]

    ################### RRHOME menu ###################

    rr_state_frame = sg.Frame(
        'State',
        [
            # show image
            [
                sg.Image(filename=f'{img_folder_path}/mini_start.png',
                         key='rr_state_img')
            ],
            [
                sg.Text('State', size=(5, 1), font='Helcetica 20'),
                sg.InputCombo(('Start', 'Restart', 'Hill_bottom', 'Hill_top',
                               'Angkor', 'Angkor_Center', 'Type2_attack',
                               'Pole_block', 'Last_attack', 'END'),
                              key='rr_state_select',
                              size=(15, 1),
                              font='Helcetica 20'),
                sg.Button('Set',
                          key='rr_set_state',
                          size=(5, 1),
                          font='Helcetica 20')
            ],
            [
                sg.Column([[
                    create_state_frame_button('Start', 'rr_start'),
                    create_state_frame_button('Restart', 'rr_restart')
                ]],
                          justification='center')
            ],
            [
                sg.Column([[
                    create_state_frame_button('Idle', 'rr_idle'),
                    create_state_frame_button('Manual', 'rr_manual')
                ]],
                          justification='center')
            ],
            [
                sg.Column([[
                    create_state_frame_button('Forward', 'rr_forward'),
                ]],
                          justification='center')
            ],
        ],
        size=(500, 800))

    rr_menu_frame = sg.Frame('INFO', [
        [
            sg.Column([[
                create_menu_frame_button(f'{rr_hard_id[0x01]}', 'rr_01'),
                create_menu_frame_button(f'{rr_hard_id[0x02]}', 'rr_02'),
                create_menu_frame_button(f'{rr_hard_id[0x03]}', 'rr_03'),
                create_menu_frame_button(f'{rr_hard_id[0x04]}', 'rr_04')
            ]],
                      justification='center')
        ],
        [
            sg.Column([[
                create_menu_frame_button(f'{rr_hard_id[0x05]}', 'rr_05'),
                create_menu_frame_button(f'{rr_hard_id[0x06]}', 'rr_06'),
                create_menu_frame_button(f'{rr_hard_id[0x07]}', 'rr_07'),
                create_menu_frame_button(f'{rr_hard_id[0x08]}', 'rr_08')
            ]],
                      justification='center')
        ],
        [
            sg.Column([[
                create_menu_frame_button(f'{rr_hard_id[0x09]}', 'rr_09'),
                create_menu_frame_button(f'{rr_hard_id[0x0A]}', 'rr_0A'),
                create_menu_frame_button(f'{rr_hard_id[0x0B]}', 'rr_0B'),
                create_menu_frame_button(f'{rr_hard_id[0x0C]}', 'rr_0C')
            ]],
                      justification='center')
        ],
        [
            sg.Column([[
                create_menu_frame_button(f'{rr_hard_id[0x0D]}', 'rr_0D'),
                create_menu_frame_button(f'{rr_hard_id[0x0E]}', 'rr_0E'),
                create_menu_frame_button(f'{rr_hard_id[0x0F]}', 'rr_0F'),
                create_menu_frame_button(f'{rr_hard_id[0x10]}', 'rr_10')
            ]],
                      justification='center')
        ],
        [
            sg.Column([[
                sg.Text('PATH', size=(4, 1), font='Helcetica 10'),
                sg.InputCombo(
                    ('1', '2', '3', '4', '5', '6', '7', '8', '9', '10'),
                    key='rr_path',
                    size=(3, 1),
                    font='Helcetica 10'),
                sg.Button('Forward',
                          key='rr_path_forward',
                          font='Helvetica 10',
                          size=(7, 2)),
                sg.Button('Backward',
                          key='rr_path_backward',
                          font='Helvetica 10',
                          size=(7, 2))
            ]],
                      justification='center')
        ],
    ],
                             size=(500, 800))

    rrhome = [[
        sg.Text('RR',
                font='Helvetica 20',
                size=(10, 2),
                justification='center')
    ], [rr_state_frame, rr_menu_frame]]

    ################### total layout ###################
    layout = [
        [sg.Column(header, key='-HEADER-', justification='center')],
        [
            sg.Column(home, key='-1-', justification='center'),
            sg.Column(erhome, visible=False, key='-2-',
                      justification='center'),
            sg.Column(rrhome, visible=False, key='-3-',
                      justification='center'),
        ],
    ]

    window = sg.Window('NHK2023 LocalGUI',
                       layout,
                       resizable=True,
                       size=(1000, 800))
    current_layout = 1
    event, values = None, None

    def __init__(self, node):
        self.node: Node = node

    # function
    def gui_check(self):
        self.event, self.values = self.window.read(timeout=10)
        print(self.event, self.values)

        if self.event in (None, 'Exit'):
            self.window.close()
            exit()

        if self.event == 'ER':
            self.window[f'-{self.current_layout}-'].update(visible=False)
            self.current_layout = 2
            self.window[f'-{self.current_layout}-'].update(visible=True)
        if self.event == 'RR':
            self.window[f'-{self.current_layout}-'].update(visible=False)
            self.current_layout = 3
            self.window[f'-{self.current_layout}-'].update(visible=True)
        if self.event == 'Home':
            self.window[f'-{self.current_layout}-'].update(visible=False)
            self.current_layout = 1
            self.window[f'-{self.current_layout}-'].update(visible=True)

    def rr_img_change(self, rr_state_img):
        # update rr state image
        self.window['rr_state_img'].update(
            filename=f'{img_folder_path}/mini_{rr_state_img}.png')

    def ping_gui(self, msg: Ping):
        for i in msg.devices:

            if (not hasattr(self.window,
                            f'rr_{format(i.hard_id,"02x").upper()}')):
                continue

            if i.is_active is True:
                self.window[f'rr_{format(i.hard_id,"02x").upper()}'].update(
                    button_color=(connected))
            else:
                self.window[f'rr_{format(i.hard_id,"02x").upper()}'].update(
                    button_color=(discon))
