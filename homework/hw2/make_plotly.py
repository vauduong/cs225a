import pandas as pd
import plotly.graph_objects as go

path = '/Users/vivian/cs225a/bin/hw2/simviz_log_files/PANDA_simviz__2025-04-24__15-38-55.csv'

df = pd.read_csv(path, sep=r'\s*,\s*',
                           header=0, encoding='ascii', engine='python')

fig = go.Figure(go.Scatter(x = df['time'], y = df['jointpositions6'],
                  name='time vs joint 7 position'))

fig.update_layout(title=dict(text='time vs joint 7 position'),
                   plot_bgcolor='rgb(230, 230,230)',
                   showlegend=False)

fig.show()