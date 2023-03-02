from graphviz import Digraph
# png
G = Digraph('state_machine', filename='test.gv', format='png')

G.attr("node")
G.node('START')
G.node('ReSTART')
G.node('Hill_top')
G.node('Hill_bottom')
G.node('Angkor')
G.node('Angkor_Center')
G.node('Type2_attack')
G.node('Pole_block')
G.node('Last_attack')
G.node('END')

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
G.render('test')