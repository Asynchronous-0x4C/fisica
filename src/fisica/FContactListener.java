package fisica;

import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.collision.Manifold;

public interface FContactListener {
  public void beginContact(FContact contact);

  public void endContact(FContact contact);

  public void preSolve(FContact contact,Manifold manifold);

  public void postSolve(FContact result,ContactImpulse impulse);
}
