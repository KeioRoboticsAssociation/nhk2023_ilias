import PySimpleGUI as sg

# GUI class
class GUI:
  sg.theme('DarkBrown2')

  header = [
      [sg.Button('Home', font='Helvetica 10',size=(10,2)) ,sg.Text('Ilias2023', font='Helvetica 30', size=(20, 2), justification='center'), sg.Button(key='emergency_stop',image_filename='img/emergency_stop.png', image_size=(100, 100), button_color=(sg.theme_background_color(), sg.theme_background_color()), )]
  ]
  home = [
      [sg.Button('RR',font='Helvetica 20', size=(10, 2)),sg.Button('ER',font='Helvetica 20', size=(10, 2))],
  ]
  # ERHOME menu
  er_state_frame=sg.Frame('State',[
          # text box to choose state
          [sg.Text('State', size=(10, 1)), sg.InputCombo(('Start', 'Restart', 'Climb','Precision'),key='state', size=(20, 1))],
      ],size=(500, 800))
  er_menu_frame=sg.Frame('Menu',[[

      ]],size=(500, 800))

  erhome = [
      [sg.Text('ER',font='Helvetica 20', size=(10, 2), justification='center')],
      [er_state_frame,er_menu_frame],
  ]

  # RRHOME menu
  rrhome = [
      [sg.Text('RR',font='Helvetica 20', size=(10, 2), justification='center')],
  ]

  layout = [
      [sg.Column(header, key='-HEADER-',justification='center')],
      [sg.Column(home, key='-1-',justification='center'),sg.Column(erhome, visible=False, key='-2-',justification='center'),sg.Column(rrhome, visible=False, key='-3-',justification='center'),],
  ]

  window = sg.Window('NHK2023 LocalGUI', layout, resizable=True, size=(1000, 800))
  current_layout = 1
  event, values = window.read()

  # function
  def gui_check(self):
      self.event, self.values = self.window.read()
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

