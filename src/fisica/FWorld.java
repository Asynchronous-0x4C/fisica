package fisica;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.ArrayList;
import java.awt.event.MouseEvent;

import org.jbox2d.common.*;
import org.jbox2d.collision.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.*;
import org.jbox2d.dynamics.joints.*;

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
  public FBox left;
  public FBox right;
  public FBox top;
  public FBox bottom;

  protected float m_edgesFriction = 0.5f;
  protected float m_edgesRestitution = 0.5f;
  protected boolean m_grabbable = true;
  protected int m_mouseButton = MouseEvent.BUTTON1;
  protected HashMap m_contacts;
  protected ArrayList m_contactResults;

  protected FMouseJoint m_mouseJoint = new FMouseJoint((FBody)null, 0.0f, 0.0f);

  private Vec2 m_small = new Vec2(0.001f, 0.001f);
  private AABB m_aabb = new AABB();

  /**
   * Forward the contact events to the contactStarted(ContactPoint point),
   * contactPersisted(ContactPoint point) and contactStopped(ContactPoint point)
   * which may be implemented in the sketch.
   *
   */
  class ConcreteContactListener implements ContactListener {
    public void add(ContactPoint point) {
      FContact contact = new FContact(point);
      m_world.m_contacts.put(contact.getId(), contact);

      if (m_world.m_contactStartedMethod == null) {
        return;
      }

      try {
        m_world.m_contactStartedMethod.invoke(Fisica.parent(),
                                              new Object[] { contact });
      } catch (Exception e) {
        System.err.println("Disabling contactStarted(ContactPoint point) because of an error.");
        e.printStackTrace();
        m_world.m_contactStartedMethod = null;
      }
    }

    public void persist(ContactPoint point) {
      FContact contact = new FContact(point);
      m_world.m_contacts.put(contact.getId(), contact);

      if (m_world.m_contactPersistedMethod == null) {
        return;
      }

      try {
        m_world.m_contactPersistedMethod.invoke(Fisica.parent(),
                                                new Object[] { contact });
      } catch (Exception e) {
        System.err.println("Disabling contactPersisted(FContact point) because of an error.");
        e.printStackTrace();
        m_world.m_contactPersistedMethod = null;
      }
    }

    public void remove(ContactPoint point) {
      FContact contact = new FContact(point);
      m_world.m_contacts.remove(contact.getId());

      if (m_world.m_contactEndedMethod == null) {
        return;
      }

      try {
        m_world.m_contactEndedMethod.invoke(Fisica.parent(),
                                            new Object[] { contact });
      } catch (Exception e) {
        System.err.println("Disabling contactEnded(FContact point) because of an error.");
        e.printStackTrace();
        m_world.m_contactEndedMethod = null;
      }
    }

    public FWorld m_world;

    public void result(ContactResult point) {
      FContactResult result = new FContactResult(point);
      m_contactResults.add(result);

      if (m_world.m_contactResultMethod == null) {
        return;
      }

      try {
        m_world.m_contactResultMethod.invoke(Fisica.parent(),
                                            new Object[] { result });
      } catch (Exception e) {
        System.err.println("Disabling contactResult(FContactResult result) because of an error.");
        e.printStackTrace();
        m_world.m_contactResultMethod = null;
      }
    }
  }

  private ConcreteContactListener m_contactListener;
  private Method m_contactStartedMethod;
  private Method m_contactPersistedMethod;
  private Method m_contactEndedMethod;
  private Method m_contactResultMethod;

  protected void mouseEvent(MouseEvent event){

    // mousePressed
    if (event.getID() == event.MOUSE_PRESSED
        && event.getButton() == m_mouseButton
        && (m_mouseJoint.getGrabbedBody() == null)) {

      FBody body = this.getBody(event.getX(), event.getY(), true);
      if ( body == null ) return;
      if (!(body.m_grabbable)) return;

      m_mouseJoint.setGrabbedBodyAndTarget(body, event.getX(), event.getY());
      m_mouseJoint.setFrequency(3.0f);
      m_mouseJoint.setDamping(0.1f);
      this.add(m_mouseJoint);
      // TODO: send a bodyGrabbed(FBody body) event
    }

    // mouseReleased
    if (event.getID() == event.MOUSE_RELEASED
        && event.getButton() == m_mouseButton
        && (m_mouseJoint.getGrabbedBody() != null)) {
      this.remove(m_mouseJoint);
      m_mouseJoint.releaseGrabbedBody();
      // TODO: send a bodyReleased(FBody body) event
    }

    // mouseDragged
    if (event.getID() == event.MOUSE_DRAGGED
        && (m_mouseJoint.getGrabbedBody() != null)) {
      m_mouseJoint.setTarget(event.getX(), event.getY());

      // TODO: send a bodyDragged(FBody body) event
    }
  }

  /**
   * Constructs the world where all the bodies live in.
   */
  public FWorld() {
    super(new AABB(Fisica.screenToWorld(new Vec2(-Fisica.parent().width,
                                                 -Fisica.parent().height)),
                   Fisica.screenToWorld(new Vec2(Fisica.parent().width,
                                                 Fisica.parent().height))),
          Fisica.screenToWorld(new Vec2(0.0f, 10.0f)),                  // gravity vertical downwards 10 m/s^2
          true);                                   // allow sleeping bodies

    Fisica.parent().registerMouseEvent(this);

    // Get the contactStarted(), contactPersisted() and contactEnded()
    // methods from the sketch
    try {
      m_contactStartedMethod =
        Fisica.parent().getClass().getMethod("contactStarted",
                                             new Class[] { FContact.class });
    } catch (Exception e) {
      // no such method, or an error.. which is fine, just ignore
    }

    try {
      m_contactPersistedMethod =
        Fisica.parent().getClass().getMethod("contactPersisted",
                                             new Class[] { FContact.class });
    } catch (Exception e) {
      // no such method, or an error.. which is fine, just ignore
    }

    try {
      m_contactEndedMethod =
        Fisica.parent().getClass().getMethod("contactEnded",
                                             new Class[] { FContact.class });
    } catch (Exception e) {
      // no such method, or an error.. which is fine, just ignore
    }

    try {
      m_contactResultMethod =
        Fisica.parent().getClass().getMethod("contactResult",
                                             new Class[] { FContactResult.class });
    } catch (Exception e) {
      // no such method, or an error.. which is fine, just ignore
    }

    m_contactListener = new ConcreteContactListener();
    m_contactListener.m_world = this;
    setContactListener(m_contactListener);

    m_contacts = new HashMap();
    m_contactResults = new ArrayList();

    m_mouseJoint.setDrawable(false);
  }

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
      Fisica.parent().registerMouseEvent(this);
    } else {
      Fisica.parent().unregisterMouseEvent(this);
    }
  }

  /**
   * Draws all the bodies in the world.  This method is often called in the draw method of the applet.
   *
   * @param applet  applet to which to draw the world.  Useful when trying to draw the world on other Processing backends, such as PDF
   * @see FBody
   */
  public void draw( PApplet applet ) {
    for (Body b = getBodyList(); b != null; b = b.m_next) {
      FBody fb = (FBody)(b.m_userData);
      if (fb != null && fb.isDrawable()) fb.draw(applet);
    }

    for (Joint j = getJointList(); j != null; j = j.m_next) {
      FJoint fj = (FJoint)(j.m_userData);
      if (fj != null && fj.isDrawable()) fj.draw(applet);
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

  public void add( FBody body ) {
    body.addToWorld(this);
  }

  public void remove( FBody body ) {
    body.removeFromWorld();
  }

  public void add( FJoint joint ) {
    joint.addToWorld(this);
  }

  public void remove( FJoint joint ) {
    joint.removeFromWorld();
  }
  public void clear() { }

  public void setDamping( float damping ) { }

  public void setEdges(PApplet applet, int color) {
    left = new FBox(20, applet.height);
    left.setStaticBody(true);
    left.setGrabbable(false);
    left.setRestitution(0.5f);
    left.setFillColor(color);
    left.setStrokeColor(color);
    left.setPosition(-5, applet.height/2);
    add(left);

    right = new FBox(20, applet.height);
    right.setStaticBody(true);
    right.setGrabbable(false);
    right.setRestitution(0.5f);
    right.setPosition(applet.width+5, applet.height/2);
    right.setFillColor(color);
    right.setStrokeColor(color);
    add(right);

    top = new FBox(applet.width, 20);
    top.setStaticBody(true);
    top.setGrabbable(false);
    top.setRestitution(0.5f);
    top.setPosition(applet.width/2, -5);
    top.setFillColor(color);
    top.setStrokeColor(color);
    add(top);

    bottom = new FBox(applet.width, 20);
    bottom.setStaticBody(true);
    bottom.setGrabbable(false);
    bottom.setRestitution(0.5f);
    bottom.setPosition(applet.width/2, applet.height+5);
    bottom.setFillColor(color);
    bottom.setStrokeColor(color);
    add(bottom);

    setEdgesFriction(m_edgesFriction);
    setEdgesRestitution(m_edgesRestitution);
  }

  public void setEdges() {
    setEdges(Fisica.parent(), 0);
  }

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

  public void setGravity( float gx, float gy ) {
    setGravity(Fisica.screenToWorld(new Vec2(gx, gy)));
  }

  public void step() {
    step(1.0f/60.0f);
  }

  public void step( float dt ) {
    step(dt, 10);
  }

  public void step( float dt, int iterationCount) {
    //m_contacts.clear();
    m_contactResults.clear();

    super.setWarmStarting( true );
    super.setPositionCorrection( true );
    super.setContinuousPhysics( true );

    super.step( dt, iterationCount );
  }

  public FBody getBody( float x, float y ) {
    return this.getBody(x, y, true);
  }

  public FBody getBody( float x, float y, boolean getStatic ) {
    ArrayList bodies = this.getBodies(x, y, getStatic);
    if (bodies.size() == 0) return null;

    return (FBody)bodies.get(0);
  }

  public ArrayList getBodies( float x, float y ) {
    return this.getBodies(x, y, true);
  }

  public ArrayList getBodies( float x, float y, boolean getStatic ) {
    return this.getBodies(x, y, getStatic, 10);
  }

  public ArrayList getBodies( float x, float y, boolean getStatic, int count ) {
    // Make a small box.
    Vec2 p = Fisica.screenToWorld(x, y);

    m_aabb.lowerBound.set(p);
    m_aabb.lowerBound.subLocal(m_small);
    m_aabb.upperBound.set(p);
    m_aabb.upperBound.addLocal(m_small);

    // Query the world for overlapping shapes.
    Shape shapes[] = this.query(m_aabb, count);

    ArrayList result = new ArrayList();

    if (shapes == null) return result;

    for (int j = 0; j < shapes.length; j++) {
      Body shapeBody = shapes[j].getBody();
      if (shapeBody.isStatic() == false || getStatic) {
        boolean inside = shapes[j].testPoint(shapeBody.getMemberXForm(), p);
        if (inside) {
          result.add((FBody)(shapeBody.getUserData()));
        }
      }
    }

    return result;
  }

}
