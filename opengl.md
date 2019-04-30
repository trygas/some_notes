1、opengl中Draw影响的是最终传出的数据范围，比如，如果输入一个矩形，再Draw一个四分之一大小的矩形，那么最终获得的数据只会出现在那四分之一的矩形范围中，其他的数据则都为0.

2、gl_FragCoord是个vec4，四个分量分别对应x，y，z，1/w。其中，x和y是当前片元的窗口相对坐标。不过这些数不是整数，小数部分恒为0.5。x-0.5与y-0.5分别位于[0, windowWidth - 1]和[0, windowHeight - 1]内。windowWidth和windowHeight都以像素为单位，亦即用glViewPort指定的宽高。w即为乘过了投影矩阵之后点坐标的w，用于perspective divide的那个值。gl_FragCoord.z / gl_FragCoord.w可以得到当前片元和camera之间的距离。

gl_FragCoord.z是固定管线计算出的当前片元的深度。它已经考虑了多边形偏移，并经过了投影变换。它位于[0.0, 1.0]之间。如果用gl_FragColor = vec4(vec3(gl_FragCoord.z), 1.0)将其可视化，多半会看到一片白。这是由于变换的非线性，大多数点的深度都非常接近于1。用gl_FragColor = vec4(vec3(pow(gl_FragColor.z, exp)), 1.0)并将exp取为合适的值，就能看到从黑到白的深度变化了。距离观察者近的颜色深，接近0.0；距离观察者远的颜色浅，接近1.0；这说明一直以来的右手坐标系在投影变换后变成了左手坐标系。