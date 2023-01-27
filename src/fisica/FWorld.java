/*
  Part of the Fisica library - http://www.ricardmarxer.com/fisica

  Copyright (c) 2009 - 2010 Ricard Marxer

  Fisica is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.
  
  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

package fisica;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.LinkedList;
import processing.event.MouseEvent;

import org.jbox2d.common.*;
import org.jbox2d.collision.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.*;
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.callbacks.*;

import processing.core.*;

/**
 * Represents the world where all the bodies live in.
 * When we create a world it will have the size of the applet that it is created in.  Once the world is created we can add bodies to it or remove bodies we have added.
 *
 * Once the world is created we may add or remove bodies to it using {@link #add(FBody) add} and  {@link #remove(FBody) remove}.  We may also call {@link #step() step} to advance one step the simulation.  Finally we can draw the world and all the bodies living in it by using  {@link #draw() draw}.
 *
 *
 *
 * <pre>
 * {@code
 * FWorld world;
 *
 * void setup() {
 *   Fisica.init(this);
 *
 *   world = new FWorld();
 *   world.setEdges();
 *
 *   // Create and add bodies to the world here
 *   // ...
 * }
 *
 * void draw() {
 *   world.step();
 *   world.draw();
 * }
 *}
 * </pre>
 *
 * @usage World
 * @see FBody
 */
public class FWorld extends World {
  /**
   * The left edge of the world.  For this edge to exist, the edges must have been created by calling {@link #setEdges()}.
   * @see #left
   * @see #right
   * @see #bottom
   * @see #top
   */
  public FBox left;

  /**
   * The right edge of the world.  For this edge to exist, the edges must have been created by calling {@link #setEdges()}.
   * @see #left
   * @see #right
   * @see #bottom
   * @see #top
   */
  public FBox right;

  /**
   * The top edge of the world.  For this edge to exist, the edges must have been created by calling {@link #setEdges()}.
   * @see #left
   * @see #right
   * @see #bottom
   * @see #top
   */
  public FBox top;

  /**
   * The bottom edge of the world.  For this edge to exist, the edges must have been created by calling {@link #setEdges()}.
   * @see #left
   * @see #right
   * @see #bottom
   * @see #top
   */
  public FBox bottom;

  private TimeStep copy_step;

  protected float m_topLeftX;
  protected float m_topLeftY;
  protected float m_bottomRightX;
  protected float m_bottomRightY;

  protected float m_edgesFriction = 0.1f;
  protected float m_edgesRestitution = 0.1f;
  protected boolean m_grabbable = true;
  protected float m_grabPositionX = 0.0f;
  protected float m_grabPositionY = 0.0f;
  protected int m_mouseButton = PConstants.LEFT;
  protected HashMap<FContactID,FContact> m_contacts;

  protected LinkedList<FWorldAction> m_actions;

  protected ArrayList<FBody> m_fbodies = new ArrayList<>();

  protected FMouseJoint m_mouseJoint = new FMouseJoint((FBody)null, 0.0f, 0.0f);

  private Vec2 m_small = new Vec2(0.001f, 0.001f);
  private AABB m_aabb = new AABB();

  public void addBody(FBody body) {
      if (body == null) { return; }

      m_fbodies.add(body);
      body.addToWorld(this);
  }

  public void removeBody(FBody body) {
      if (body == null) { return; }
      
      if (body == m_mouseJoint.getGrabbedBody()) {
        this.removeJoint(m_mouseJoint);
        m_mouseJoint.releaseGrabbedBody();
      }

      m_fbodies.remove(body);
      body.removeFromWorld();  
  }

  public void addJoint(FJoint joint) {
      if (joint == null) { return; }

      joint.addToWorld(this);
  }

  public void removeJoint(FJoint joint) {
      if (joint == null) { return; }

      joint.removeFromWorld();
  }
  
  
  /**
   * Forward the contact events to the beginContact(Contact contact),
   * endContact(Contact contact) and contactStopped(ContactPoint point)
   * which may be implemented in the sketch.
   *
   */
  class ConcreteContactListener implements ContactListener {
    public void beginContact(Contact contact) {
      FContact f_contact = new FContact(contact);
      m_world.m_contacts.put(f_contact.getId(), f_contact);
      
      if (m_clientContactListener != null) {
        try {
          m_clientContactListener.beginContact(f_contact);
          return;
        } catch (Exception e) {
          System.err.println("Disabling contact listener because of an error.");
          e.printStackTrace();
          m_clientContactListener = null;
        }
      }

      if (m_world.m_beginContactMethod == null) {
        return;
      }

      try {
        m_world.m_beginContactMethod.invoke(Fisica.parent(),f_contact);
      } catch (Exception e) {
        System.err.println("Disabling contactStarted(ContactPoint point) because of an error.");
        e.printStackTrace();
        m_world.m_beginContactMethod = null;
      }
    }

    public void endContact(Contact contact) {
      FContact f_contact = new FContact(contact);
      m_world.m_contacts.remove(f_contact.getId());

      if (m_clientContactListener != null) {
        try {
          m_clientContactListener.endContact(f_contact);
          return;
        } catch (Exception e) {
          System.err.println("Disabling contact listener because of an error.");
          e.printStackTrace();
          m_clientContactListener = null;
        }
      }

      if (m_world.m_endContactMethod == null) {
        return;
      }

      try {
        m_world.m_endContactMethod.invoke(Fisica.parent(),f_contact );
      } catch (Exception e) {
        System.err.println("Disabling contactPersisted(FContact point) because of an error.");
        e.printStackTrace();
        m_world.m_endContactMethod = null;
      }
    }

    public void preSolve(Contact point,Manifold manifold) {
      FContact contact = new FContact(point);

      if (m_clientContactListener != null) {
        try {
          m_clientContactListener.preSolve(contact,manifold);
          return;
        } catch (Exception e) {
          System.err.println("Disabling contact listener because of an error.");
          e.printStackTrace();
          m_clientContactListener = null;
        }
      }

      if (m_world.m_preSolveMethod == null) {
        return;
      }

      try {
        m_world.m_preSolveMethod.invoke(Fisica.parent(),contact,manifold);
      } catch (Exception e) {
        System.err.println("Disabling contactEnded(FContact point) because of an error.");
        e.printStackTrace();
        m_world.m_preSolveMethod = null;
      }
    }

    public FWorld m_world;

    public void postSolve(Contact point,ContactImpulse impulse) {
      FContact f_contact = new FContact(point);

      if (m_clientContactListener != null) {
        try {
          m_clientContactListener.postSolve(f_contact,impulse);
          return;
        } catch (Exception e) {
          System.err.println("Disabling contact listener because of an error.");
          e.printStackTrace();
          m_clientContactListener = null;
        }
      }
      
      if (m_world.m_postSolveMethod == null) {
        return;
      }

      try {
        m_world.m_postSolveMethod.invoke(Fisica.parent(),f_contact,impulse);
      } catch (Exception e) {
        System.err.println("Disabling contactResult(FContactResult result) because of an error.");
        e.printStackTrace();
        m_world.m_postSolveMethod = null;
      }
    }
  }

  private ConcreteContactListener m_contactListener;
  
  /**
   * A PApplet can either provide an {@link FContactListener} for receiving contact events,
   * or can implement the contact event methods itself without explicitly implementing
   * {@link FContactListener}.
   */
  private FContactListener m_clientContactListener;
  private Method m_beginContactMethod;
  private Method m_endContactMethod;
  private Method m_preSolveMethod;
  private Method m_postSolveMethod;

  
  public void setContactListener(final FContactListener listener) {
    m_clientContactListener = listener;
  }
  
  public void grabBody(float x, float y) {
    if (m_mouseJoint.getGrabbedBody() != null) {
        return;
    }
    
      FBody body = this.getBody(x, y, true);
      if ( body == null ) return;
      if (!(body.m_grabbable)) return;


      m_mouseJoint.setGrabbedBodyAndTarget(body, x, y);
      m_mouseJoint.setFrequency(3.0f);
      m_mouseJoint.setDamping(0.1f);
      this.addJoint(m_mouseJoint);

      m_grabPositionX = x - body.getX();
      m_grabPositionY = y - body.getY();
  }
  
  public void dragBody(float x, float y) {
    if (m_mouseJoint.getGrabbedBody() == null) {
        return;
    }
    
    FBody body = m_mouseJoint.getGrabbedBody();
    if (body.isStatic()) {
        body.setPosition(x - m_grabPositionX, 
                         y - m_grabPositionY);
    } else {
        m_mouseJoint.setTarget(x, y);
    }

  }
  
  public void releaseBody() {
    if (m_mouseJoint.getGrabbedBody() == null) {
        return;
    }
    
    this.removeJoint(m_mouseJoint);
    m_mouseJoint.releaseGrabbedBody();
  }

  /**
   * This is an internal method to handle mouse interaction and should not be used.
   * @internal
   * @exclude
   */
  public void mouseEvent(MouseEvent event){
    // mousePressed
    if (event.getAction() == MouseEvent.PRESS
        && event.getButton() == m_mouseButton) {
      
      grabBody(event.getX(), event.getY());
      // TODO: send a bodyGrabbed(FBody body) event
    }

    // mouseReleased
    if (event.getAction()  == MouseEvent.RELEASE
        && event.getButton() == m_mouseButton) {
        
      releaseBody();
      // TODO: send a bodyReleased(FBody body) event
    }

    // mouseDragged
    if (event.getAction()  == MouseEvent.DRAG) {
    
      dragBody(event.getX(), event.getY());
      // TODO: send a bodyDragged(FBody body) event
    }
  }

  /**
   * Constructs the world where all the bodies live in.  We usually want to build the world larger than the actual screen,
   * because when bodies exit the world they will appear stuck since they do not get update anymore.  By default the world's
   * width and height are three times larger than those of the Processing canvas.
   *
   * {@code
   FWorld world;
   
   void setup() {
     size(200, 200);

     Fisica.init(this);
     world = new FWorld(-width, -height, 2*width, 2*height);
   }
   *}
   *
   * @usage World
   * @param topLeftX  the horizontal coordinate of the top left corner of the world
   * @param topLeftY  the vertical coordinate of the top left corner of the world
   * @param bottomRightX  the horizontal coordinate of the bottom right corner of the world
   * @param bottomRightY  the vertical coordinate of the bottom right corner of the world
   * @see FBody
   */
  public FWorld(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY) {
    super(Fisica.screenToWorld(new Vec2(0.0f, 10.0f)));

    m_topLeftX = topLeftX;
    m_topLeftY = topLeftY;
    m_bottomRightX = bottomRightX;
    m_bottomRightY = bottomRightY;

    super.setWarmStarting( true );
    //super.setPositionCorrection( true ); 
    super.setContinuousPhysics( true );

    //Fisica.parent().registerMouseEvent(this);

    Fisica.parent().registerMethod("mouseEvent",this);
    
    // Get the contactStarted(), contactPersisted() and contactEnded()
    // methods from the sketch
    try {
      m_beginContactMethod =
        Fisica.parent().getClass().getMethod("beginContact",
                                             new Class[] { FContact.class });
    } catch (Exception e) {
      // no such method, or an error.. which is fine, just ignore
    }

    try {
      m_endContactMethod =
        Fisica.parent().getClass().getMethod("endContact",
                                             new Class[] { FContact.class });
    } catch (Exception e) {
      // no such method, or an error.. which is fine, just ignore
    }

    try {
      m_preSolveMethod =
        Fisica.parent().getClass().getMethod("preSolve",FContact.class,Manifold.class);
    } catch (Exception e) {
      // no such method, or an error.. which is fine, just ignore
    }

    try {
      m_postSolveMethod =
        Fisica.parent().getClass().getMethod("postSolve",FContactResult.class,ContactImpulse.class);
    } catch (Exception e) {
      // no such method, or an error.. which is fine, just ignore
    }

    m_contactListener = new ConcreteContactListener();
    m_contactListener.m_world = this;
    setContactListener(m_contactListener);

    m_contacts = new HashMap<>();
    
    m_actions = new LinkedList<>();

    m_mouseJoint.setDrawable(false);

    setGravity(0, 200);

    try{
      Field f=World.class.getDeclaredField("step");
      copy_step=(TimeStep)f.get(this);
    }catch(Exception e){
      e.printStackTrace();
    }
  }

  public FWorld() {
    this(-Fisica.parent().width, -Fisica.parent().height, 2*Fisica.parent().width, 2*Fisica.parent().height);
  }

  /**
   * Returns the mouse joint that is used for interaction with the bodies in the world.
   *
   * @return the mouse joint used for grabbing bodies
   */
  public FMouseJoint getMouseJoint() {
    return m_mouseJoint;
  }

  /**
   * Controls whether the bodies in the world can be grabbed by the mouse or not.  By default the world bodies' are grabbable and draggable.
   *
   * {@code
   world.setGrabbable(false);
   * }
   *
   * @usage World
   * @param value  if true the bodies that live in this world can be grabbed and dragged using the mouse
   * @see FBody
   */
  public void setGrabbable(boolean value) {
    if (m_grabbable == value) return;

    m_grabbable = value;
    if (m_grabbable) {
      //Fisica.parent().registerMouseEvent(this);
      Fisica.parent().registerMethod("mouseEvent",this);
    } else {
    	
      //Fisica.parent().unregisterMouseEvent(this);

      Fisica.parent().unregisterMethod("mouseEvent",this);
    }
  }

  public void processActions() {
    while (m_actions.size()>0) {
      ((FWorldAction)m_actions.poll()).apply(this);
    }
  }

  /**
   * Draws all the bodies in the world.  This method is often called in the draw method of the applet.
   *
   * @param applet  applet to which to draw the world.  Useful when trying to draw the world on other Processing backends, such as PDF
   * @see FBody
   */
  public void draw( PApplet applet ) {
    draw(applet.g);
  }

  /**
   * Draws all the bodies in the world.  This method is often called in the draw method of the applet.
   *
   * @param graphics  graphics to which to draw the world.  Useful when trying to draw the world on other buffers, such as when using createGraphics
   * @see FBody
   */
  public void draw( PGraphics graphics ) {
    processActions();
    /*
    for (Body b = getBodyList(); b != null; b = b.m_next) {
      FBody fb = (FBody)(b.m_userData);
      if (fb != null && fb.isDrawable()) fb.draw(applet);
    }
    */

    for (int i=0; i<m_fbodies.size(); i++) {
      FBody fb = (FBody)m_fbodies.get(i);
      if (fb != null && fb.isDrawable()) fb.draw(graphics);
    }
    
    for (Joint j = getJointList(); j != null; j = j.m_next) {
      FJoint fj = (FJoint)(j.m_userData);
      if (fj != null && fj.isDrawable()) fj.draw(graphics);
    }
  }

  /**
   * Draws the debug version of all the bodies in the world.  This method is often called in the draw method of the applet.
   *
   * @param applet  applet to which to draw the world.  Useful when trying to draw the world on other Processing backends, such as PDF
   * @see FBody
   */
  public void drawDebug( PApplet applet ) {
    drawDebug(applet.g);
  }

  
  /**
   * Draws the debug version of all the bodies in the world.  This method is often called in the draw method of the applet.
   *
   * @param graphics  graphics to which to draw the world.  Useful when trying to draw the world on other buffers, such as when using createGraphics
   * @see FBody
   */
  public void drawDebug( PGraphics graphics ) {
    processActions();
    /*
    for (Body b = getBodyList(); b != null; b = b.m_next) {
      FBody fb = (FBody)(b.m_userData);
      if (fb != null && fb.isDrawable()) fb.draw(applet);
    }
    */

    for (int i=0; i<m_fbodies.size(); i++) {
      FBody fb = (FBody)m_fbodies.get(i);
      if (fb != null) fb.drawDebug(graphics);
    }
    
    for (Joint j = getJointList(); j != null; j = j.m_next) {
      FJoint fj = (FJoint)(j.m_userData);
      if (fj != null) fj.drawDebug(graphics);
    }
  }
  


  /**
   * Draws all the bodies in the world on the applet canvas.  This method is often called in the draw method of the applet.
   *
   * @see FBody
   */
  public void draw() {
    draw(Fisica.parent());
  }

  /**
   * Draws the debug version of all the bodies in the world on the applet canvas.  This method is often called in the draw method of the applet.
   *
   * @see FBody
   */
   public void drawDebug() {
    drawDebug(Fisica.parent());
  }

  /**
   * Add a body to the world.
   *
   * @param body   body to be added to the world
   * @see #remove(FBody)
   */
  public void add( FBody body ) {
    FWorldAction action = new FAddBodyAction(body);
    m_actions.add(action);
  }

  /**
   * Remove a body from the world.
   *
   * @param body   body to be removed from the world
   * @see #add(FBody)
   */
  public void remove( FBody body ) {
    FWorldAction action = new FRemoveBodyAction(body);
    m_actions.add(action);
  }

  /**
   * Add a joint to the world.
   *
   * @param joint   joint to be added to the world
   * @see #remove(FJoint)
   */
  public void add( FJoint joint ) {
    FWorldAction action = new FAddJointAction(joint);
    m_actions.add(action);
  }

  /**
   * Remove a joint from the world.
   *
   * @param joint   joint to be removed from the world
   * @see #add(FJoint)
   */
  public void remove( FJoint joint ) {
    FWorldAction action = new FRemoveJointAction(joint);
    m_actions.add(action);
  }
  
  /**
   * Clear all bodies and joints from the world.  NOT IMPLEMENTED YET.
   *
   */
  public void clear() { 
    for (Joint j = getJointList(); j != null; j = j.m_next) {
      FJoint fj = (FJoint)(j.m_userData);
      remove(fj);
    }

    for (Body b = getBodyList(); b != null; b = b.m_next) {
      FBody fb = (FBody)(b.m_userData);
      remove(fb);
    }
  }

  /**
   * Add edges of given dimensions to the world. This will create the bodies for {@link #left}, {@link #right}, {@link #bottom} and {@link #top}.
   *
   * @param topLeftX  the horizontal coordinate of the top left corner of the edges
   * @param topLeftY  the vertical coordinate of the top left corner of the edges
   * @param bottomRightX  the horizontal coordinate of the bottom right corner of the edges
   * @param bottomRightY  the vertical coordinate of the bottom right corner of the edges
   * @param color  the color of the edges.  This color must be passed using Processing's color() function
   */
  public void setEdges(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY, int color) {
    float height = Math.abs(bottomRightY - topLeftY);
    float width = Math.abs(bottomRightX - topLeftX);

    float ymid = (topLeftY + bottomRightY)/2.0f;
    float xmid = (topLeftX + bottomRightX)/2.0f;

    left = new FBox(20, height);
    left.setStaticBody(true);
    left.setGrabbable(false);
    left.setFillColor(color);
    left.setStrokeColor(color);
    left.setPosition(topLeftX-5, ymid);
    addBody(left);

    right = new FBox(20, height);
    right.setStaticBody(true);
    right.setGrabbable(false);
    right.setFillColor(color);
    right.setStrokeColor(color);
    right.setPosition(bottomRightX+5, ymid);
    addBody(right);

    top = new FBox(width, 20);
    top.setStaticBody(true);
    top.setGrabbable(false);
    top.setFillColor(color);
    top.setStrokeColor(color);
    top.setPosition(xmid, topLeftY-5);
    addBody(top);

    bottom = new FBox(width, 20);
    bottom.setStaticBody(true);
    bottom.setGrabbable(false);
    bottom.setFillColor(color);
    bottom.setStrokeColor(color);
    bottom.setPosition(xmid, bottomRightY+5);
    addBody(bottom);

    setEdgesFriction(m_edgesFriction);
    setEdgesRestitution(m_edgesRestitution);
  }

  /**
   * Add black edges of given dimensions to the world. This will create the bodies for {@link #left}, {@link #right}, {@link #bottom} and {@link #top}.
   *
   * @param topLeftX  the horizontal coordinate of the top left corner of the edges
   * @param topLeftY  the vertical coordinate of the top left corner of the edges
   * @param bottomRightX  the horizontal coordinate of the bottom right corner of the edges
   * @param bottomRightY  the vertical coordinate of the bottom right corner of the edges
   * @param color  the color of the edges.  This color must be passed using Processing's color() function
   */  
  public void setEdges(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY) {
    setEdges(topLeftX, topLeftY, bottomRightX, bottomRightY, Fisica.parent().color(0));
  }

  /**
   * Add edges of given applet's dimensions to the world. This will create the bodies for {@link #left}, {@link #right}, {@link #bottom} and {@link #top}.
   *
   * @param applet  applet from where to get the dimensions for the edges
   * @param color  the color of the edges.  This color must be passed using Processing's color() function
   */
  public void setEdges(PApplet applet, int color) {
    setEdges(0, 0, applet.width, applet.height, color);
  }

  /**
   * Add edges of Processing's canvas dimensions to the world. This will create the bodies for {@link #left}, {@link #right}, {@link #bottom} and {@link #top}.
   *
   * @param color  the color of the edges.  This color must be passed using Processing's color() function
   */
  public void setEdges(int color) {
    setEdges(Fisica.parent(), color);
  }

  /**
   * Add black edges of Processing's canvas dimensions to the world. This will create the bodies for {@link #left}, {@link #right}, {@link #bottom} and {@link #top}.
   */
  public void setEdges() {
    setEdges(Fisica.parent(), Fisica.parent().color(0));
  }

  /**
   * Set the friction of all the edges.
   *
   * @param friction   the friction of the edges
   */
  public void setEdgesFriction( float friction ) {
    if (left != null) {
      left.setFriction(friction);
    }

    if (right != null) {
      right.setFriction(friction);
    }

    if (top != null) {
      top.setFriction(friction);
    }

    if (bottom != null) {
      bottom.setFriction(friction);
    }

    m_edgesFriction = friction;
  }

  /**
   * Set the restitution of all the edges.
   *
   * @param restitution   the restitution of the edges
   */
  public void setEdgesRestitution( float restitution ) {
    if (left != null) {
      left.setRestitution(restitution);
    }

    if (right != null) {
      right.setRestitution(restitution);
    }

    if (top != null) {
      top.setRestitution(restitution);
    }

    if (bottom != null) {
      bottom.setRestitution(restitution);
    }

    m_edgesRestitution = restitution;
  }

  /**
   * Set the gravity of the world. Use {@code world.setGravity(0,0);} to have a world without gravity.
   *
   * @param gx   the horizontal component of the gravity
   * @param gy   the vertical component of the gravity
   */
  public void setGravity( float gx, float gy ) {
    setGravity(Fisica.screenToWorld(new Vec2(gx, gy)));
  }

  /**
   * Advance the world simulation of 1/60th of a second.
   */
  public void step() {
    step(1.0f/60.0f);
  }

  /**
   * Advance the world simulation of given time.
   *
   * @param dt   the time to advance the world simulation
   */
  public void step( float dt ) {
    step(dt, 10);
  }

  /**
   * Advance the world simulation of given time, with a given number of iterations.  The larger the number of iterations, the more computationally expensive and precise it will be.  The default is 10 iterations.
   *
   * @param dt   the time to advance the world simulation
   * @param iterationCount   the number of iterations for the world simulation step
   */
  public void step( float dt, int iterationCount) {
    processActions();
    
    //m_contacts.clear();

    super.step( dt, iterationCount, iterationCount );
  }

  /**
   * Advance the world simulation of given time, with a given number of iterations.  The larger the number of iterations, the more computationally expensive and precise it will be.  The default is 10 iterations.
   *
   * @param dt   the time to advance the world simulation
   * @param velocityIterationCount   the number of iterations for the world simulation step (velocity)
   * @param positionIterationCount   the number of iterations for the world simulation step (position)
   */
  public void step( float dt, int velocityIterationCount, int positionIterationCount) {
    processActions();
    
    //m_contacts.clear();

    super.step( dt, velocityIterationCount, positionIterationCount );
  }

  public float getInv_dt(){
    return copy_step.inv_dt;
  }

  /**
   * Returns the first object found at the given position.
   *
   * @param x   the horizontal component of the position
   * @param y   the vertical component of the position
   * @return    the body found at the given position
   */
  public FBody getBody( float x, float y ) {
    return this.getBody(x, y, true);
  }

  /**
   * Returns the first object found at the given position.
   *
   * @param x   the horizontal component of the position
   * @param y   the vertical component of the position
   * @param getStatic   if {@code true} it will also get static objects that can be found at that position
   * @return    the body found at the given position
   */
  public FBody getBody( float x, float y, boolean getStatic ) {
    ArrayList<FBody> bodies = this.getBodies(x, y, getStatic);
    if (bodies.size() == 0) return null;

    return (FBody)bodies.get(0);
  }

  /**
   * Returns a list with all the bodies in the world
   *
   * @return    an ArrayList (of FBody) of the bodies in the world
   */
  public ArrayList<FBody> getBodies() {
      ArrayList<FBody> result = new ArrayList<>();

      for (Body b = getBodyList(); b != null; b = b.m_next) {
	  FBody fb = (FBody)(b.m_userData);
          if (fb != null) {
            result.add(fb);
          }
      }
      
      return result;
  }

  /**
   * Returns a list with the 10 first bodies found at the given position.
   *
   * @param x   the horizontal component of the position
   * @param y   the vertical component of the position
   * @return    an ArrayList (of FBody) of the 10 first bodies found at the given position
   */
  public ArrayList<FBody> getBodies( float x, float y ) {
    return this.getBodies(x, y, true);
  }

  /**
   * Returns a list with the 10 first bodies found at the given position.
   *
   * @param x   the horizontal component of the position
   * @param y   the vertical component of the position
   * @param getStatic   if {@code true} it will also get static objects that can be found at that position
   * @return    an ArrayList (of FBody) of the 10 first bodies found at the given position
   */
  public ArrayList<FBody> getBodies( float x, float y, boolean getStatic ) {
    return this.getBodies(x, y, getStatic, 10);
  }

  /**
   * Returns a list with all the bodies found at the given position.
   *
   * @param x   the horizontal component of the position
   * @param y   the vertical component of the position
   * @param getStatic   if {@code true} it will also get static objects that can be found at that position
   * @param count   the maximum number of bodies to be retrieved
   * @return    an ArrayList (of FBody) of all bodies found at the given position
   */

  public ArrayList<FBody> getBodies( float x, float y, boolean getStatic, int count ) {
    ArrayList<FBody> result = new ArrayList<>();

    if (count <= 0) return result;

    // Make a small box.
    Vec2 p = Fisica.screenToWorld(x, y);

    m_aabb.lowerBound.set(p);
    m_aabb.lowerBound.subLocal(m_small);
    m_aabb.upperBound.set(p);
    m_aabb.upperBound.addLocal(m_small);

    // Query the world for overlapping shapes.
    ArrayList<Fixture>fixtures=new ArrayList<>();
    // Count and add shapes.
    this.queryAABB((f)->{fixtures.add(f);return fixtures.size()<count;},(i)->{return true;},m_aabb);

    for(Fixture f:fixtures){
      Body shapeBody = f.getBody();
      if(shapeBody.getType() != BodyType.STATIC || getStatic){
        if(f.getShape().testPoint(shapeBody.getTransform(),p)){
          result.add((FBody)(shapeBody.getUserData()));
        }
      }
    }

    return result;
  }
  
  @Deprecated
  public int raycast(float x1, float y1, float x2, float y2, FBody[] bodies, int maxCount, boolean solidShapes) {

    int[] count = {0};

    raycast((Fixture fixture,Vec2 point,Vec2 normal,float fraction)->{count[0]++;return 1;},(int index,Vec2 point,Vec2 normal,float fraction)->{return -1;},x1,x2,y1,y2);
    
    return count[0];
  }

  @Deprecated
  public FBody raycastOne(float x1, float y1, float x2, float y2, FRaycastResult result, boolean solidShapes) {
    Fixture[] ret={new Fixture()};

    result.m_x1=x1;
    result.m_x2=x2;
    result.m_y1=y1;
    result.m_y2=y2;

    Vec2[] v={new Vec2()};

    float[] l={0};

    raycast((Fixture fixture,Vec2 point,Vec2 normal,float fraction)->{l[0]=(point.x-x1)/(x2-x1);v[0]=normal;ret[0]=fixture;return 0;},(int index,Vec2 point,Vec2 normal,float fraction)->{return -1;},x1,x2,y1,y2);

    result.m_lambda=l[0];
    result.m_normal=v[0];

    //We already know it returns true
    return (FBody)(ret[0].getBody().getUserData());
  }

  public void raycast(RayCastCallback rc,ParticleRaycastCallback pc,float x1, float y1, float x2, float y2){
    this.raycast(rc, pc, Fisica.screenToWorld(x1, y1), Fisica.screenToWorld(x2, y2));
  }

}
