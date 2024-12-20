# Importing modules...
import plotly.graph_objects as go
import numpy as np

class Viewer2D: 
    def __init__(self, title='', resolution=(720,720), image=None, graphical=False):
        self.title = title
        self.resolution = resolution # Change feed dimensions 
        self.graphical = graphical # Toggle to activate graphical mode

        if image is not None:
            if image.ndim == 2: # For single channel images
                image = np.repeat(np.expand_dims(image, axis=2), 3, axis=2) # Replicate the channels 3 times (Plotly only admits RGB data)

        # Create Figure 
        self.figure = go.Figure(
            data=go.Image(z=image), 
            layout=go.Layout(
                height=700, 
                width=700, 
                title=go.layout.Title(text=self.title)
            )

        )

        self.figure.update_yaxes(
            scaleanchor='x',
            scaleratio=1
        )
        
        self.figure.update_layout(
            xaxis_title='u',
            yaxis_title='v',
            plot_bgcolor='white',
            font=dict(
                family='Arial',
                size=15,
                color='black'
            ),
            xaxis=dict(
                gridcolor='lightgray',
                dtick = resolution[0]/10,
                range=[0, self.resolution[0]]
            ),
            yaxis=dict(
                gridcolor='lightgray',
                dtick = resolution[1]/10,
                range=[self.resolution[1], 0]
            )
        )

        self.figure.add_shape(
            type='rect',
            x0=0, y0=0, x1=resolution[0], y1=resolution[1],
            line=dict(color='black'),
        )

    def add_points(self, points, name, color=None, colorscale=None, range=None, colorbar=None):
        self.figure.add_trace(
            go.Scatter(
                x=points[0],
                y=points[1],
                mode='markers',
                marker=dict(
                    size=5,
                    opacity=0.80,
                    color=color,
                    cmin=range[0] if colorbar is not None else None,
                    cmax=range[1] if colorbar is not None else None,
                    colorbar=dict(title=colorbar,lenmode='fraction', len=0.5) if colorbar is not None else colorbar,
                    colorscale=colorscale
                ),
                name=name,
                legendgroup='Points',
                legendgrouptitle_text='Points',
                showlegend=self.graphical
            )
        )

    def add_lines(self, lines, name=None, color=None):
        for line_number, line in enumerate(lines):
            a, b, c = line
            x, y = self.resolution[0], self.resolution[1]

            points = [[   0, -c/a,          x, (-c-b*y)/a],
                      [-c/b,    0, (-c-a*x)/b,          y]]
            
            self.figure.add_trace(
                go.Scatter(
                    x=points[0],
                    y=points[1],
                    mode='lines',
                    line=dict(color=color, 
                              width=1),
                    name=name,
                    legendgroup='Lines',
                    legendgrouptitle_text='Lines',
                    showlegend=self.graphical if not line_number else False
                )
            )

