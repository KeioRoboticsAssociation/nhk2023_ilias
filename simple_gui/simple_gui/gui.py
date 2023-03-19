import PySimpleGUI as sg
from ament_index_python.packages import get_package_share_directory
from rogilink2_interfaces.msg import Ping

img_folder_path = get_package_share_directory('simple_gui') + '/img'
rr_hard_id = (0,"FR_S","FR","FL_S","FL","RR_S","RR","RL_S","RL",0,0,0,0,"limit_sw","steer_sensor","sensor","LED")
discon = ('White', 'Black')
connected = ('Black', 'White')

# GUI class
class GUI:


  sg.theme('Topanga')
  ################### Header ###################
  header = [
      [sg.Button('Home', font='Helvetica 10',size=(10,2)) ,
       sg.Text('Ilias2023', font='Helvetica 30', size=(20, 2), justification='center'),
       sg.Button(key='emergency_stop',image_filename=f'{img_folder_path}/emergency_stop.png', image_size=(100, 100), button_color=(sg.theme_background_color(), sg.theme_background_color()), )]
  ]
  home = [
      [sg.Button('RR',font='Helvetica 20', size=(10, 2)),sg.Button('ER',font='Helvetica 20', size=(10, 2))],
  ]

  ################### ERHOME menu ###################
  er_state_frame=sg.Frame('State',[
          # text box to choose state
          [sg.Text('State', size=(10, 1)),
           sg.InputCombo(('Start', 'Restart', 'Hill_bottom','Hill_top','Angkor','Angkor_Center','Type2_attack','Pole_block','Last_attack','END'),key='state', size=(20, 1)),
           sg.Button('Set', key='set_state', size=(10, 1))],
          [sg.Button('Start', key='start', size=(10, 1)), sg.Button('Restart', key='restart', size=(10, 1))],
          [sg.Button('Idele', key='idele', size=(10, 1)), sg.Button('Manual', key='manual', size=(10, 1))],
      ],size=(500, 800))
  er_menu_frame=sg.Frame('Menu',[[

      ]],size=(500, 800))

  erhome = [
      [sg.Text('ER',font='Helvetica 20', size=(10, 2), justification='center')],
      [er_state_frame,er_menu_frame],
  ]

  ################### RRHOME menu ###################
  rr_state_frame=sg.Frame('State',[
          # show image
          [sg.Image(filename=f'{img_folder_path}/mini_start.png', key='rr_state_img')],
          [sg.Text('State', size=(5, 1),font='Helcetica 20'),
           sg.InputCombo(('Start', 'Restart', 'Hill_bottom','Hill_top','Angkor','Angkor_Center','Type2_attack','Pole_block','Last_attack','END'),
          key='rr_state_select', size=(15, 1),font='Helcetica 20'), sg.Button('Set', key='rr_set_state', size=(5, 1),font='Helcetica 20')],
          [sg.Column([[sg.Button('Start', key='rr_start', font='Helvetica 20',size=(10,2)), sg.Button('Restart', key='rr_restart', font='Helvetica 20',size=(10,2))]], justification='center')],
          [sg.Column([[sg.Button('Idle', key='rr_idle', font='Helvetica 20',size=(10,2)), sg.Button('Manual', key='rr_manual', font='Helvetica 20',size=(10,2))]], justification='center')],
          [sg.Column([[sg.Button('Forward', key='rr_forward', font='Helvetica 20',size=(10,2)),]], justification='center')],
    ],size=(500, 800))

  rr_menu_frame=sg.Frame('INFO',[
      [sg.Column([[sg.Button(f'{rr_hard_id[0x01]}', key='rr_01', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x02]}', key='rr_02', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x03]}', key='rr_03', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x04]}', key='rr_04', font='Helvetica 20',size=(4,2),button_color=(discon))] ], justification='center')],
      [sg.Column([[sg.Button(f'{rr_hard_id[0x05]}', key='rr_05', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x06]}', key='rr_06', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x07]}', key='rr_07', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x08]}', key='rr_08', font='Helvetica 20',size=(4,2),button_color=(discon))] ], justification='center')],
      [sg.Column([[sg.Button(f'{rr_hard_id[0x09]}', key='rr_09', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x0A]}', key='rr_0A', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x0B]}', key='rr_0B', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x0C]}', key='rr_0C', font='Helvetica 20',size=(4,2),button_color=(discon))] ], justification='center')],
      [sg.Column([[sg.Button(f'{rr_hard_id[0x0D]}', key='rr_0D', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x0E]}', key='rr_0E', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x0F]}', key='rr_0F', font='Helvetica 20',size=(4,2),button_color=(discon)),sg.Button(f'{rr_hard_id[0x10]}', key='rr_10', font='Helvetica 20',size=(4,2),button_color=(discon))] ], justification='center')],
      [sg.Column([[sg.Text('PATH', size=(4, 1),font='Helcetica 10'),sg.InputCombo(('1', '2', '3', '4', '5', '6', '7', '8', '9', '10'), key='rr_path', size=(3, 1),font='Helcetica 10'),sg.Button('Forward', key='rr_path_forward', font='Helvetica 10',size=(7,2)),sg.Button('Backward', key='rr_path_backward', font='Helvetica 10',size=(7,2))]], justification='center')],
  ],size=(500, 800))

  rrhome = [
      [sg.Text('RR',font='Helvetica 20', size=(10, 2), justification='center')],
      [rr_state_frame,rr_menu_frame]
  ]

  ################### total layout ###################
  layout = [
      [sg.Column(header, key='-HEADER-',justification='center')],
      [sg.Column(home, key='-1-',justification='center'),sg.Column(erhome, visible=False, key='-2-',justification='center'),sg.Column(rrhome, visible=False, key='-3-',justification='center'),],
  ]

  window = sg.Window('NHK2023 LocalGUI', layout, resizable=True, size=(1000, 800))
  current_layout = 1
  event, values = None , None

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



  def rr_img_change(self,rr_state_img):
      # update rr state image
      self.window['rr_state_img'].update(filename=f'{img_folder_path}/mini_{rr_state_img}.png')

  def ping_gui(self,msg:Ping):
      for i in msg.devices:
          if i.is_active == True:
              self.window[f'rr_{format(i.hard_id,"02x").upper()}'].update(button_color=(connected))
          else:
              self.window[f'rr_{format(i.hard_id,"02x").upper()}'].update(button_color=(discon))








