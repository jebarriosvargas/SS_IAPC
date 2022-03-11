#!/usr/bin/env python
# coding: utf-8

# # Producto cruz de dos vectores

# Consideremos dos vectores, $\vec{P}$ y $\vec{Q}$, de los cuales conocemos sus componentes cartesianas. Es decir, tenemos que
# 
# \begin{align*}
# \vec{P} = (P_x,P_y,P_z)\,,
# \end{align*}
# 
# \begin{align*}
# \vec{Q} = (Q_x,Q_y,Q_z)\,.
# \end{align*}
# 
# Alternativamente, podemos escribir $\vec{P} = P_x\hat{\imath}+P_y\hat{\jmath}+P_z\hat{k}$ y $\vec{Q} = Q_x\hat{\imath}+Q_y\hat{\jmath}+Q_z\hat{k}$; donde $\hat{\imath}$, $\hat{\jmath}$ y $\hat{k}$ son los vectores unitarios en la dirección de $X$, $Y$ y $Z$ respectivamente.
# 
# El producto cruz de dos vectores da como resultado otro vector, $\vec{R}=\vec{P}\times\vec{Q}$, tal que es perpendicular a los otros dos vectores y cuya norma es igual al área del paralelogramo formado por los vectores $\vec{P}$ y $\vec{Q}$. La elección de la dirección perpendicular sigue la regla de la mano derecha.
# 
# Las componentes del vector resultante del producto cruz pueden obtenerse calculando el siguiente determinante,
# \begin{align*}
# \vec{R} =& \left| \begin{array}{ccc} 
# \hat{\imath} & \hat{\jmath} & \hat{k} \\
# P_x & P_y & P_z \\
# Q_x & Q_y & Q_z 
# \end{array} \right|\\
# =& 
# \left|\begin{array}{cc}
# P_y&P_z\\
# Q_y&Q_z
# \end{array}\right| \hat{\imath}
# -\left|\begin{array}{cc}
# P_x&P_z\\
# Q_x&Q_z
# \end{array}\right|\hat{\jmath}
# +\left|\begin{array}{cc}
# P_x&P_y\\
# Q_x&Q_y
# \end{array}\right|\hat{k}\\
# =& (P_yQ_z-P_zQ_y)\hat{\imath}
# -(P_xQ_z-P_zQ_x)\hat{\jmath}
# +(P_xQ_y-P_yQ_x)\hat{k}
# \end{align*}
# 
# Alternativamente podemos escribir el vector $\vec{R}$ por:
# \begin{align*}
# \vec{R} = \big( P_yQ_z-P_zQ_y, P_zQ_x-P_xQ_z, P_xQ_y-P_yQ_x\big)
# \end{align*}
# 
# A continuación se muestra el producto cruz tomando como ejemplo los siguiente vectores:
# 
# \begin{align*}
# \vec{P}&=(2,1,-3)\\
# \vec{Q}&=(-1,2,2)\\
# \vec{R}&=\big( 1\cdot2-(-3)\cdot2 ,\; (-3)\cdot(-1)-2\cdot2,\; 2\cdot2-1\cdot(-1) \big)\\
# &=(2+6,\;3-4,\;4+1)\\
# &=(8,-1,5)
# \end{align*}
# 
# En el gráfico siguiente note que el vector $\vec{R}$ sigue la regla de la mano derecha.

# In[1]:


from pylab import *
import plotly.graph_objects as go


# In[2]:


P = array( [ 2,1,-3] )
Q = array( [-1,2,2] )
R = cross(P,Q)


# In[3]:


# Colores de los elementos a graficar
ColorP = 'rgba(0, 191, 255, 60)'
ColorQ = 'rgba(238, 130, 238, 60)'
ColorR = 'rgba(255, 255, 255, 60)'
TextoR = r"$\vec{{R}}=({0:.1f},{1:.1f},{2:.1f})$".format(R[0],R[1],R[2])
ColorEjes = 'rgba(255, 210, 0, 60)'

# Parámetros para ajustar la graficación
vecs     = column_stack((P,Q,R))
ValorMax = vecs.max()*1.1 # Máximo en los ejes
ValorMin = vecs.min()*1.1 # Mínimo en los ejes
# Tamaños de las puntas de los vectores
Tip   = 1.0
Shift = 0.2*Tip
ShiftR = 1*Tip
# Dirección de los vectores
Punit = P/sqrt(dot(P,P))
Qunit = Q/sqrt(dot(Q,Q))
Runit = R/sqrt(dot(R,R))

# Vector P
TipP = go.Cone(x=[P[0]],y=[P[1]],z=[P[2]],
               u=[Tip*Punit[0]],v=[Tip*Punit[1]],w=[Tip*Punit[2]],
               colorscale=[[0, ColorP],[1.0, ColorP]],
               sizemode="absolute",
               sizeref=1.0,
               anchor="tip",
               showscale=False)
VecP = go.Scatter3d(x=[0,P[0]-Tip*Punit[0]],
                    y=[0,P[1]-Tip*Punit[1]],
                    z=[0,P[2]-Tip*Punit[2]],
                    mode="lines",showlegend=False,
                    line=dict(width=10,color=ColorP))
# Vector Q
TipQ = go.Cone(x=[Q[0]],y=[Q[1]],z=[Q[2]],
               u=[Tip*Qunit[0]],v=[Tip*Qunit[1]],w=[Tip*Qunit[2]],
               colorscale=[[0, ColorQ],[1.0, ColorQ]],
               sizemode="absolute",
               sizeref=1.0,
               anchor="tip",
               showscale=False)
VecQ = go.Scatter3d(x=[0,Q[0]-Tip*Qunit[0]],
                    y=[0,Q[1]-Tip*Qunit[1]],
                    z=[0,Q[2]-Tip*Qunit[2]],
                    mode="lines",showlegend=False,
                    line=dict(width=10,color=ColorQ))
# Vector R
TipR = go.Cone(x=[R[0]],y=[R[1]],z=[R[2]],
               u=[Tip*Runit[0]],v=[Tip*Runit[1]],w=[Tip*Runit[2]],
               colorscale=[[0, ColorR],[1.0, ColorR]],
               sizemode="absolute",
               sizeref=1.0,
               anchor="tip",
               showscale=False)
VecR = go.Scatter3d(x=[0,R[0]-Tip*Runit[0]],
                    y=[0,R[1]-Tip*Runit[1]],
                    z=[0,R[2]-Tip*Runit[2]],
                    mode="lines",showlegend=False,
                    line=dict(width=10,color=ColorR))
# Puntas de los ejes
ConosEjes = go.Cone( x = [ValorMax,0,0],
                     y = [0,ValorMax,0],
                     z = [0,0,ValorMax],
                     u = [Tip, 0, 0],
                     v = [0, Tip, 0],
                     w = [0, 0, Tip],
                     colorscale=[[0, ColorEjes],[1.0, ColorEjes]],
                     sizemode="absolute",
                     sizeref=0.1,
                     anchor="tip",
                     showscale=False)
# Eje x
Eje_x = go.Scatter3d(x = [ValorMin,ValorMax-Tip],
                     y = [0,0],
                     z = [0,0],
                     mode = 'lines',
                     showlegend = False,
                     line = dict(width = 6,
                                 color = ColorEjes))
# Eje y
Eje_y = go.Scatter3d(x = [0,0],
                     y = [ValorMin,ValorMax-Tip],
                     z = [0,0],
                     mode = 'lines',
                     showlegend = False,
                     line = dict(width = 6,
                                 color = ColorEjes))
# Eje z
Eje_z = go.Scatter3d(x = [0,0],
                     y = [0,0],
                     z = [ValorMin,ValorMax-Tip],
                     mode = 'lines',
                     showlegend = False,
                     line = dict(width = 6,
                                 color = ColorEjes)) 
# --------------------------------------------------
# Graficación de los diferentes elementos
fig = go.Figure(data=[ConosEjes,Eje_x,Eje_y,Eje_z,
                      TipP,VecP,
                      TipQ,VecQ,
                      TipR,VecR])
# Ajustes de la escena
fig.update_layout( # - begin scene                  
scene = dict(xaxis_title='',yaxis_title='',zaxis_title='',
    xaxis = dict(nticks=0, range=[ValorMin,ValorMax],ticktext=[""],tickvals= [0.0],visible=False),
    yaxis = dict(nticks=0, range=[ValorMin,ValorMax],ticktext=[""],tickvals= [0.0],visible=False),
    zaxis = dict(nticks=0, range=[ValorMin,ValorMax],ticktext=[""],tickvals= [0.0],visible=False),
    camera_eye=dict(x=1.5, y=0.8, z=0.6),
    annotations=[\
      dict(showarrow=False, x=ValorMax, y=Shift, z=Shift,
           text="$X$",xanchor="left",xshift=0,
           opacity=1.0,font=dict(size=40,color = ColorEjes) ),
      dict(showarrow=False, x=Shift,y=ValorMax,z=Shift,
           text="$Y$",xanchor="left",xshift=0,
           opacity=1.0,font=dict(size=40,color = ColorEjes) ),
      dict(showarrow=False, x=Shift,y=Shift,z=ValorMax,
           text="$Z$",xanchor="left",xshift=0,
           opacity=1.0,font=dict(size=40,color = ColorEjes)),
      dict(showarrow=False,x=P[0]+Shift,y=P[1]+Shift,z=P[2]+Shift,
           text=r"$\vec{P}$",xanchor="center",
           opacity=1.0,font=dict(size=50,color=ColorP)),
      dict(showarrow=False,x=Q[0]+Shift,y=Q[1]+Shift,z=Q[2]+Shift,
           text=r"$\vec{Q}$",xanchor="center",
           opacity=1.0,font=dict(size=50,color=ColorQ)),
      dict(showarrow=False,
           x=R[0]-ShiftR*Punit[0],
           y=R[1]-ShiftR*Punit[1],
           z=R[2]-ShiftR*Punit[2],
           text=TextoR,
           xanchor="center",
           opacity=1.0,font=dict(size=60,color=ColorR))
    ]), 
# - end scene
template="plotly_dark",
width = 600, height= 600)

# Mostrar la gráfica
fig.show()


# In[ ]:





# In[ ]:




