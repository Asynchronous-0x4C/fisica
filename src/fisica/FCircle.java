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

import org.jbox2d.collision.shapes.*;
import org.jbox2d.dynamics.*;

import processing.core.*;

/**
 * Represents a circular body that can be added to a world.
 * <pre>
 * {@code
 * FCircle myCircle = new FCircle(40);
 * world.add(myCircle);
 * }
 * </pre>
 *
 * @usage Bodies
 * @see FBox
 * @see FBlob
 * @see FPoly
 * @see FLine
 */
public class FCircle extends FBody {
  protected float m_size;

  protected FixtureDef getFixtureDef() {
    CircleShape cs = new CircleShape();
    cs.m_radius = m_size/2.0f;
    FixtureDef fd=new FixtureDef();
    fd.shape=cs;
    fd.density = m_density;
    fd.friction = m_friction;
    fd.restitution = m_restitution;
    fd.isSensor = m_sensor;
    return fd;
  }

  protected FixtureDef getTransformedFixtureDef() {
    FixtureDef fd = getFixtureDef();
    ((CircleShape)fd.getShape()).m_p.set(m_position);
    return fd;
  }

  /**
   * Constructs a circular body that can be added to a world.
   *
   * @param size  the size of the circle
   */
  public FCircle(float size){
    super();

    m_size = Fisica.screenToWorld(size);
  }

  /**
   * Returns the size of the circle.
   *
   * @usage Bodies
   * @return the size of the circle
   */
  public float getSize(){
    return Fisica.worldToScreen(m_size);
  }

  /**
   * Sets the size of the circle.
   * Under the hood the body is removed and readded to the world.
   *
   * @usage Bodies
   * @param size the size of the circle
   */
  public void setSize(float size){
    m_size = Fisica.screenToWorld(size);

    this.recreateInWorld();
  }

  public void draw(PGraphics applet) {
    preDraw(applet);

    if (m_image != null ) {
      drawImage(applet);
    } else {
      applet.ellipse(0, 0, getSize(), getSize());
    }

    postDraw(applet);
  }
  
  public void drawDebug(PGraphics applet) {
    preDrawDebug(applet);
        
    applet.ellipse(0, 0, getSize(), getSize());
    applet.line(0, 0, getSize()/2, 0);

    postDrawDebug(applet);
  }


}
