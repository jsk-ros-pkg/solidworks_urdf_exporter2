// ---- three.js + the loaders + the urdf-manipulator element --------------
// A LEAF module: it imports nothing (only the CDN packages), so every module
// that reads THREE at evaluation time is guaranteed to see it filled in.
// The top-level await makes every importer async -- which the whole block
// already was.
export let THREE, GLTFLoader, STLLoader, ColladaLoader, OBJLoader, mergeGeometries,
    TransformControls;
try {
  THREE = await import('three');
  ({ GLTFLoader } = await import('three/examples/jsm/loaders/GLTFLoader.js'));
  ({ STLLoader } = await import('three/examples/jsm/loaders/STLLoader.js'));
  ({ ColladaLoader } =
      await import('three/examples/jsm/loaders/ColladaLoader.js'));
  ({ OBJLoader } = await import('three/examples/jsm/loaders/OBJLoader.js'));
  ({ mergeGeometries } =
      await import('three/examples/jsm/utils/BufferGeometryUtils.js'));
  ({ TransformControls } =
      await import('three/examples/jsm/controls/TransformControls.js'));
  const { default: URDFManipulator } = await import(
    'https://unpkg.com/urdf-loader@0.12.7/src/urdf-manipulator-element.js');
  log(t('load.ok'), 'ok');
  customElements.define('urdf-manipulator', URDFManipulator);
} catch (e) {
  log(t('load.fail', { e: e.message ?? e }), 'err');
  log(t('load.failHint'), 'wrn');
  throw e;
}
