package fisica;

import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.collision.Manifold;

/**
 * Provides default do-nothing implementations of all {@link FContactListener}
 * methods.
 */
public class FContactAdapter implements FContactListener {

  @Override
  public void beginContact(FContact contact) {
  }

  @Override
  public void endContact(FContact contact) {
  }

  @Override
  public void preSolve(FContact contact,Manifold manifold) {
  }

  @Override
  public void postSolve(FContact result,ContactImpulse impulse) {
  }

}
