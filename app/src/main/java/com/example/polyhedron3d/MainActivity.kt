package com.example.polyhedron3d

import android.app.Activity
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.os.Bundle
import android.view.Gravity
import android.view.MotionEvent
import android.widget.Button
import android.widget.LinearLayout
import android.widget.TextView
import android.widget.FrameLayout
import kotlin.math.*
import java.nio.ByteBuffer
import java.nio.ByteOrder
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

private data class V3(val x:Float,val y:Float,val z:Float) {
 operator fun plus(b:V3)=V3(x+b.x,y+b.y,z+b.z)
 operator fun minus(b:V3)=V3(x-b.x,y-b.y,z-b.z)
 operator fun times(s:Float)=V3(x*s,y*s,z*s)
 fun dot(b:V3)=x*b.x+y*b.y+z*b.z
 fun cross(b:V3)=V3(y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x)
 fun length()=sqrt(dot(this))
 fun unit():V3 { val l=length();return if(l>1e-6f)this*(1f/l) else V3(0f,0f,0f) }
}
private data class Mesh(val vertices:List<V3>,val faces:List<IntArray>) {
 val edges=faces.flatMap{f->f.indices.map{i->val a=f[i];val b=f[(i+1)%f.size];if(a<b)a to b else b to a}}.distinct()
 companion object {
  fun make(k:Int):Mesh=when(k%3){
   0->Mesh(listOf(V3(1f,1f,1f),V3(-1f,-1f,1f),V3(-1f,1f,-1f),V3(1f,-1f,-1f)),listOf(intArrayOf(0,1,2),intArrayOf(0,3,1),intArrayOf(0,2,3),intArrayOf(1,3,2)))
   1->Mesh(listOf(V3(-1f,-1f,-1f),V3(1f,-1f,-1f),V3(1f,1f,-1f),V3(-1f,1f,-1f),V3(-1f,-1f,1f),V3(1f,-1f,1f),V3(1f,1f,1f),V3(-1f,1f,1f)),listOf(intArrayOf(0,1,2,3),intArrayOf(4,7,6,5),intArrayOf(0,4,5,1),intArrayOf(3,2,6,7),intArrayOf(0,3,7,4),intArrayOf(1,5,6,2)))
   else->Mesh(listOf(V3(1f,0f,0f),V3(-1f,0f,0f),V3(0f,1f,0f),V3(0f,-1f,0f),V3(0f,0f,1f),V3(0f,0f,-1f)),listOf(intArrayOf(0,2,4),intArrayOf(2,1,4),intArrayOf(1,3,4),intArrayOf(3,0,4),intArrayOf(2,0,5),intArrayOf(1,2,5),intArrayOf(3,1,5),intArrayOf(0,3,5)))
  }
 }
}
private class Body(val id:Int,var kind:Int,var pos:V3,var vel:V3,val color:FloatArray) {
 var rot=V3(0f,0f,0f);var spin=V3(.7f+id*.13f,.9f-id*.08f,.5f+id*.11f);val scale=.43f
 val mesh get()=Mesh.make(kind)
 fun world(v:V3):V3 {
  val sx=sin(rot.x);val cx=cos(rot.x);val sy=sin(rot.y);val cy=cos(rot.y);val sz=sin(rot.z);val cz=cos(rot.z)
  val x1=v.x;val y1=v.y*cx-v.z*sx;val z1=v.y*sx+v.z*cx
  val x2=x1*cy+z1*sy;val y2=y1;val z2=-x1*sy+z1*cy
  return V3((x2*cz-y2*sz)*scale+pos.x,(x2*sz+y2*cz)*scale+pos.y,z2*scale+pos.z)
 }
 fun vertices()=mesh.vertices.map(::world)
}
private class Engine {
 val bodies=mutableListOf<Body>();var count=4;var gravity=false;var paused=false
 private val colors=arrayOf(floatArrayOf(1f,.22f,.35f),floatArrayOf(.08f,.82f,.55f),floatArrayOf(1f,.62f,.08f),floatArrayOf(.58f,.35f,1f),floatArrayOf(.12f,.65f,1f))
 init{reset()}
 fun reset(){bodies.clear();repeat(count){i->val a=2*PI*i/count;bodies.add(Body(i,i%3,V3((cos(a)*.9).toFloat(),(sin(a)*.9).toFloat(),if(i%2==0)-.5f else .5f),V3((sin(a)*.65).toFloat(),(-cos(a)*.55).toFloat(),if(i%2==0).48f else -.48f),colors[i%colors.size]))}}
 fun step(dt0:Float){if(paused)return;val dt=dt0.coerceIn(.001f,.025f);repeat(2){val h=dt/2f
  for(b in bodies){if(gravity)b.vel=b.vel+V3(0f,-1.1f*h,0f);b.pos=b.pos+b.vel*h;b.rot=b.rot+b.spin*h
   val maxR=b.vertices().maxOf{(it-b.pos).length()};val d=b.pos.length()+maxR
   if(d>2.45f){val n=b.pos.unit();b.pos=b.pos-n*(d-2.45f);val vn=b.vel.dot(n);if(vn>0f)b.vel=b.vel-n*(1.82f*vn)}
  }
  for(i in bodies.indices)for(j in i+1 until bodies.size)collide(bodies[i],bodies[j])
 }}
 private fun collide(a:Body,b:Body){val va=a.vertices();val vb=b.vertices();val axes=mutableListOf<V3>()
  fun faceAxes(body:Body,verts:List<V3>){for(f in body.mesh.faces)if(f.size>=3){val n=(verts[f[1]]-verts[f[0]]).cross(verts[f[2]]-verts[f[0]]).unit();if(n.length()>.5f)axes.add(n)}}
  faceAxes(a,va);faceAxes(b,vb)
  for(ea in a.mesh.edges)for(eb in b.mesh.edges){val da=(va[ea.second]-va[ea.first]).unit();val db=(vb[eb.second]-vb[eb.first]).unit();val ax=da.cross(db).unit();if(ax.length()>.02f)axes.add(ax)}
  var minO=Float.MAX_VALUE;var best=V3(0f,0f,0f)
  for(ax0 in axes){val ax=ax0.unit();val pa=va.map{it.dot(ax)};val pb=vb.map{it.dot(ax)};val o=min(pa.max(),pb.max())-max(pa.min(),pb.min());if(o<=0f)return;if(o<minO){minO=o;best=if((b.pos-a.pos).dot(ax)>=0f)ax else ax*-1f}}
  if(minO<Float.MAX_VALUE){a.pos=a.pos-best*(minO*.51f);b.pos=b.pos+best*(minO*.51f);val rel=(b.vel-a.vel).dot(best);if(rel<0f){val imp=-(1f+.82f)*rel*.5f;a.vel=a.vel-best*imp;b.vel=b.vel+best*imp}}
 }
}
private class Scene(ctx:android.content.Context):GLSurfaceView(ctx),GLSurfaceView.Renderer {
 val engine=Engine();private var program=0;private val projection=FloatArray(16);private val view=FloatArray(16);private val model=FloatArray(16);private val mvp=FloatArray(16)
 private var yaw=0f;private var pitch=18f;private var last=System.nanoTime();private var lastX=0f;private var lastY=0f
 private val vs="attribute vec4 aPosition; attribute vec4 aColor; uniform mat4 uMVP; varying vec4 vColor; void main(){gl_Position=uMVP*aPosition;vColor=aColor;}"
 private val fs="precision mediump float; varying vec4 vColor; void main(){gl_FragColor=vColor;}"
 init{setEGLContextClientVersion(2);setRenderer(this);renderMode=RENDERMODE_CONTINUOUSLY}
 override fun onSurfaceCreated(gl:GL10?,c:EGLConfig?){GLES20.glClearColor(.008f,.015f,.04f,1f);GLES20.glEnable(GLES20.GL_DEPTH_TEST);program=GLES20.glCreateProgram();val a=shader(GLES20.GL_VERTEX_SHADER,vs);val b=shader(GLES20.GL_FRAGMENT_SHADER,fs);GLES20.glAttachShader(program,a);GLES20.glAttachShader(program,b);GLES20.glLinkProgram(program)}
 private fun shader(t:Int,s:String):Int{val x=GLES20.glCreateShader(t);GLES20.glShaderSource(x,s);GLES20.glCompileShader(x);return x}
 override fun onSurfaceChanged(gl:GL10?,w:Int,h:Int){GLES20.glViewport(0,0,w,h);Matrix.perspectiveM(projection,0,48f,w.toFloat()/h.coerceAtLeast(1),.1f,30f)}
 override fun onDrawFrame(gl:GL10?){val n=System.nanoTime();engine.step(((n-last)/1e9).toFloat());last=n;GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT);Matrix.setLookAtM(view,0,0f,0f,7.4f,0f,0f,0f,0f,1f,0f);Matrix.rotateM(view,0,pitch,1f,0f,0f);Matrix.rotateM(view,0,yaw,0f,1f,0f);GLES20.glUseProgram(program);drawSphere();engine.bodies.forEach(::drawBody)}
 private fun draw(v:FloatArray,c:FloatArray,mode:Int){val p=ByteBuffer.allocateDirect(v.size*4).order(ByteOrder.nativeOrder()).asFloatBuffer();p.put(v).position(0);val cb=ByteBuffer.allocateDirect(c.size*4).order(ByteOrder.nativeOrder()).asFloatBuffer();cb.put(c).position(0);val pi=GLES20.glGetAttribLocation(program,"aPosition");val ci=GLES20.glGetAttribLocation(program,"aColor");GLES20.glEnableVertexAttribArray(pi);GLES20.glVertexAttribPointer(pi,3,GLES20.GL_FLOAT,false,0,p);GLES20.glEnableVertexAttribArray(ci);GLES20.glVertexAttribPointer(ci,4,GLES20.GL_FLOAT,false,0,cb);val vp=FloatArray(16);Matrix.multiplyMM(vp,0,projection,0,view,0);Matrix.multiplyMM(mvp,0,vp,0,model,0);GLES20.glUniformMatrix4fv(GLES20.glGetUniformLocation(program,"uMVP"),1,false,mvp,0);GLES20.glDrawArrays(mode,0,v.size/3);GLES20.glDisableVertexAttribArray(pi);GLES20.glDisableVertexAttribArray(ci)}
  private fun drawSphere() {
   Matrix.setIdentityM(model, 0)
   val vertices = mutableListOf<Float>()
   fun point(theta: Double, phi: Double): V3 {
    val x = (2.48 * cos(theta) * sin(phi)).toFloat()
    val y = (2.48 * cos(phi)).toFloat()
    val z = (2.48 * sin(theta) * sin(phi)).toFloat()
    return V3(x, y, z)
   }
   for (i in 0 until 24) {
    for (j in 0 until 16) {
     val a = 2.0 * PI * i / 24.0
     val b = PI * j / 16.0
     val c = 2.0 * PI * (i + 1) / 24.0
     val d = PI * (j + 1) / 16.0
     val segments = listOf(point(a,b), point(c,b), point(c,b), point(c,d),
      point(c,d), point(a,d), point(a,d), point(a,b))
     for (p in segments) {
      vertices.add(p.x); vertices.add(p.y); vertices.add(p.z)
     }
    }
   }
   val colors = FloatArray(vertices.size / 3 * 4) { index ->
    if (index % 4 == 3) 1f else 0.12f
   }
   draw(vertices.toFloatArray(), colors, GLES20.GL_LINES)
  }
 private fun drawBody(b:Body){Matrix.setIdentityM(model,0);val v=mutableListOf<Float>();val c=mutableListOf<Float>();val w=b.vertices();for(f in b.mesh.faces)for(i in 1 until f.size-1){val tri=listOf(w[f[0]],w[f[i]],w[f[i+1]]);val n=(tri[1]-tri[0]).cross(tri[2]-tri[0]).unit();val light=(.42f+.58f*abs(n.dot(V3(.3f,.6f,1f).unit()))).coerceIn(.25f,1f);tri.forEach{p->v.add(p.x);v.add(p.y);v.add(p.z);c.add(b.color[0]*light);c.add(b.color[1]*light);c.add(b.color[2]*light);c.add(1f)}};draw(v.toFloatArray(),c.toFloatArray(),GLES20.GL_TRIANGLES);val e=mutableListOf<Float>();val ec=mutableListOf<Float>();for(edge in b.mesh.edges)for(i in listOf(edge.first,edge.second)){val p=w[i];e.add(p.x);e.add(p.y);e.add(p.z);ec.add(1f);ec.add(1f);ec.add(1f);ec.add(1f)};draw(e.toFloatArray(),ec.toFloatArray(),GLES20.GL_LINES)}
 override fun onTouchEvent(e:MotionEvent):Boolean{when(e.actionMasked){MotionEvent.ACTION_DOWN->{lastX=e.x;lastY=e.y;return true};MotionEvent.ACTION_MOVE->{yaw+=(e.x-lastX)*.45f;pitch=(pitch+(e.y-lastY)*.35f).coerceIn(-80f,80f);lastX=e.x;lastY=e.y;return true}};return true}
}
class MainActivity:Activity(){
 private lateinit var scene:Scene;private lateinit var status:TextView
 override fun onCreate(saved:Bundle?){super.onCreate(saved);scene=Scene(this);val root=FrameLayout(this);root.addView(scene,FrameLayout.LayoutParams(-1,-1));val panel=LinearLayout(this).apply{orientation=LinearLayout.HORIZONTAL;gravity=Gravity.CENTER_VERTICAL;setPadding(8,6,8,6);setBackgroundColor(0xCC172338.toInt())}
 fun btn(s:String,act:()->Unit)=Button(this).apply{text=s;textSize=12f;setOnClickListener{act()}}
 panel.addView(btn("物体 −"){scene.queueEvent{scene.engine.count=(scene.engine.count-1).coerceAtLeast(2);scene.engine.reset();runOnUiThread{status.text="物体数："+scene.engine.count}}})
 panel.addView(btn("物体 +"){scene.queueEvent{scene.engine.count=(scene.engine.count+1).coerceAtMost(8);scene.engine.reset();runOnUiThread{status.text="物体数："+scene.engine.count}}})
 panel.addView(btn("重力"){scene.queueEvent{scene.engine.gravity=!scene.engine.gravity}})
 panel.addView(btn("暂停"){scene.queueEvent{scene.engine.paused=!scene.engine.paused}})
 panel.addView(btn("重新发射"){scene.queueEvent{scene.engine.reset()}})
 status=TextView(this).apply{text="3D 多面体碰撞";setTextColor(-1);textSize=13f;setPadding(8,0,2,0)};panel.addView(status)
 root.addView(panel,FrameLayout.LayoutParams(-1,-2,Gravity.TOP));setContentView(root)}
 override fun onPause(){super.onPause();scene.onPause()}
 override fun onResume(){super.onResume();scene.onResume()}
}