from graphviz import Digraph
# png
G = Digraph('state_machine', filename='test.dot', format='png')

G.attr("node")
# G.node('START',style='filled', fillcolor='#ff000042')
# G.node('ReSTART',style='filled', fillcolor='#ff000042')
# G.node('Hill_top',style='filled', fillcolor='#ff000042')
# G.node('Hill_bottom', style='filled',fillcolor='#ff000042')
# G.node('Angkor',style='filled', fillcolor='#ff000042')
# G.node('Angkor_Center',style='filled', fillcolor='#ff000042')
# G.node('Type2_attack', style='filled',fillcolor='#ff000042')
# G.node('Pole_block', style='filled',fillcolor='#ff000042')
# G.node('Last_attack',style='filled', fillcolor='#ff000042')
# G.node('END', style='filled',fillcolor='#ff000042')

G.edge('START', 'Hill_bottom')
G.edge('ReSTART', 'Hill_bottom')
G.edge('Hill_bottom', 'Hill_top')
G.edge('Hill_top', 'Angkor')
G.edge('Angkor', 'Angkor_Center')
G.edge('Angkor_Center', 'Type2_attack')
G.edge('Type2_attack', 'Pole_block')
G.edge('Pole_block', 'Last_attack')
G.edge('Last_attack', 'END')

# write out png file
G.render()