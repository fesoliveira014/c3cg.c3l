# C3 Bindings Authoring Guidelines

These are the conventions we settled on across the five library bindings in this project (ImGui, SDL3, Flecs, Jolt, Clay). Captured here so future binding work stays consistent.

## The core principle

The **module namespace** is what isolates the library, not the original C prefix. Every downstream call site reads `library::identifier`, and the library prefix never appears on the C3 side. The `@extern("original_c_name")` annotation is the only place the C symbol name survives, because the linker needs it.

This means no `@builtin` attributes on binding declarations — `@builtin` promotes symbols to global scope, which defeats the module namespace the binding is trying to provide. If you find yourself reaching for `@builtin` to avoid typing the module prefix, you're fighting the language; type the prefix.

## Naming conventions

**Module**: short, unique, lowercase. `sdl` not `sdl3`, `jolt` not `joltc`, `imgui`, `flecs`, `clay`. The number in a C library's version ("SDL3") belongs in documentation and marketing, not in the C3 identifier. The version is an implementation detail captured by whichever release you vendor.

**Functions and methods**: `snake_case`. This matches C3's standard library convention and keeps the language feeling uniform regardless of where a given function came from. The C-side names, whatever shape they had (PascalCase in ImGui, camelCase in some C libraries, `Class_Method` in Jolt), all normalize to snake_case on the C3 side.

**Types**: `PascalCase`, with the library prefix stripped. `ImGui_Context` becomes `imgui::Context`, `SDL_Window` becomes `sdl::Window`, `JPH_PhysicsSystem` becomes `jolt::PhysicsSystem`. If the C library uses a `_t` suffix (typical of Unix-style C conventions, e.g., Flecs's `ecs_world_t`), strip that too — `flecs::World`.

**Constants and enum values**: `SCREAMING_SNAKE_CASE`, with the library prefix stripped. `SDL_INIT_VIDEO` becomes `sdl::INIT_VIDEO`, `CLAY_ALIGN_X_CENTER` becomes `clay::AlignX.CENTER` when it fits a proper enum or `clay::ALIGN_X_CENTER` when it doesn't. Group related flag constants into C3 `enum` types when the C library treats them as a closed set.

## Prefix stripping

Strip the library prefix from all identifiers on the C3 side. Combining a module prefix with a name prefix (`sdl::SDL_Init`, `jolt::JPH_Init`) is the worst of both worlds and should never appear.

For multi-word prefixes like `SDL_GL_`, strip the whole thing at once and use a sub-module if the inner prefix represents a coherent sub-API. `SDL_GL_CreateContext` becomes `sdl::gl::create_context`, not `sdl::gl_create_context`. The sub-module namespace earns its keep by grouping related functions and constants (`sdl::gl::set_attribute`, `sdl::gl::CONTEXT_MAJOR_VERSION`, etc.) without crowding the root module.

For `Class_Method`-style names (Jolt's `JPH_PhysicsSystem_Create`), collapse them using C3 method syntax — see the section below.

## Sub-modules for optional backends

When a library has distinct backend implementations that shouldn't all be loaded at once, put each in its own sub-module. ImGui is the motivating example — the render backend and the platform backend are independent, and you pick one of each. We used:

- `imgui::gl` for the OpenGL render backend
- `imgui::sdl` for the SDL3 platform backend
- `sdl::gl` for SDL3's OpenGL-context helpers

Each sub-module declares its own module name (`module imgui::gl;`) and `import`s its parent if it needs to reference types from the root namespace. This gives you `imgui::gl::init()` / `imgui::sdl::process_event(&ev)` — natural and unambiguous.

## Method syntax for class-oriented C APIs

When a C library's function naming already follows a `Class_Method` pattern (Jolt and joltc are the textbook case), lift that structure into C3's method syntax. Declare methods directly on the type using `fn Ret Type.method(&self, ...)`:

```c3
extern fn PhysicsSystem* PhysicsSystem.create(PhysicsSystemSettings* s) @extern("JPH_PhysicsSystem_Create");
extern fn void           PhysicsSystem.update(&self, float dt, int steps) @extern("JPH_PhysicsSystem_Update");
extern fn BodyInterface* PhysicsSystem.get_body_interface(&self) @extern("JPH_PhysicsSystem_GetBodyInterface");
```

Call sites then read naturally:

```c3
jolt::PhysicsSystem* ps = jolt::PhysicsSystem.create(&settings);
ps.update(dt, 1);
jolt::BodyInterface* bi = ps.get_body_interface();
```

Constructors don't take `&self` and are invoked as `Type.method(...)` returning a new instance. Instance methods take `&self` as the first parameter and are invoked as `instance.method(...)`. The C3 compiler de-sugars `ps.update(dt, 1)` into `PhysicsSystem.update(ps, dt, 1)`, which through the `@extern` annotation calls the real C symbol. The method-style ergonomics are purely a C3 surface; the wire format is unchanged.

Where multiple concrete types in the C library share one underlying handle (Jolt's `BoxShape`, `SphereShape`, `HeightFieldShape` all produce `Shape*`), declare them as C3 aliases of the underlying type so each can carry its own `.create()` constructor:

```c3
alias BoxShape         = Shape;
alias SphereShape      = Shape;
alias HeightFieldShape = Shape;

extern fn Shape* BoxShape.create(Vec3* half_ext, float density) @extern("JPH_BoxShape_Create");
extern fn Shape* SphereShape.create(float radius, float density) @extern("JPH_SphereShape_Create");
```

Don't force method syntax onto flat C APIs that don't have a `Class_Method` structure. Flecs and SDL3 are naturally flat and stay as module-level functions. Jolt is the exception that earned the method-syntax treatment.

## Macro-heavy declarative C APIs

When a C library uses macros to provide declarative syntax (Clay's `CLAY(decl) { children }` block macros are the example), use C3's body-macro feature (`;@body` parameter) to replicate the trailing-block form without the C preprocessor tricks. The library's flat C entry points become `@extern @private` internal hooks, and the public interface is a thin C3 macro layer on top:

```c3
extern fn void __open_element() @extern("Clay__OpenElement") @private;
extern fn void __configure_open_element(ElementDeclaration decl) @extern("Clay__ConfigureOpenElement") @private;
extern fn void __close_element() @extern("Clay__CloseElement") @private;

macro @element(ElementDeclaration decl; @body) {
    __open_element();
    __configure_open_element(decl);
    @body();
    __close_element();
}
```

For small helper macros that just build struct literals, C3's arrow-macro form is the clean translation:

```c3
macro SizingAxis grow()              => { .type = SIZING_GROW };
macro SizingAxis fixed(float px)     => { .type = SIZING_FIXED, .size_value = px };
macro Padding    padding_all(ushort n) => { n, n, n, n };
```

## Scope and incrementalism

Bind only the surface you actually use. A C library's full API can be hundreds or thousands of functions; the C3 binding can start as the 30–50 functions the first milestone needs and grow as later milestones reach for more. This keeps the binding file small enough to audit and review, and it avoids committing to a surface you don't yet understand.

The trade-off is that "the binding" is never "done" — it expands across milestones. That's fine, and often desirable: each milestone's binding additions appear in the milestone doc as a focused, reviewable extension rather than hiding in a monolithic binding file nobody reads.

## `@extern` strings

The string passed to `@extern` is the real C symbol, verbatim. Always keep the library prefix, the exact casing, and the exact punctuation there — this is ABI-level and the linker uses it to resolve the symbol. Don't be alarmed seeing `SDL_`, `JPH_`, `ImGui_`, `Clay_`, `ecs_`, etc. inside `@extern("...")` strings; that's correct and necessary. The C3 identifier beside the string is what downstream code calls.

## Directory and file layout

Binding files live in `lib/<module>.c3l/` as a C3 library package, with the binding declarations in one `.c3i` or `.c3` file per sub-module. A typical layout:

```
lib/sdl.c3l/
├── manifest.json
├── sdl.c3i              <-- module sdl
├── gl.c3                <-- module sdl::gl
├── linux/libSDL3.so.0
├── macos/libSDL3.0.dylib
└── windows/SDL3.dll
```

Static libraries are platform-specific subdirectories; dynamic libraries ship with release builds via your engine's build scripts.

## Opaque types vs. exposed unions

Declare a type `@opaque` when the C3 side only needs to hold and pass pointers to it. This is the default for handles and working structs — `sdl::Window`, `jolt::PhysicsSystem`, `flecs::World`, etc.

Declare the full struct or union when the C3 side needs to read fields. SDL3's `Event` is the case in point: `sdl::poll_event(&ev)` returns and then engine code reads `ev.type`, `ev.key.scancode`, `ev.motion.xrel`, etc. — that requires the full tagged-union layout. When a type must be fully declared, declare it in the earliest binding file that passes it, even if that milestone only reads one variant — it's cleaner than declaring it opaque and then retroactively "fleshing it out" a few milestones later.

Match the C struct layout byte-for-byte: field order, field widths, padding. Test by comparing `@sizeof(T)` on the C3 side against `sizeof(T)` on the C side in a small diagnostic program. Size and layout mismatches between C3 and C will corrupt memory silently, and they're the hardest binding bugs to diagnose.

## Engine-side helpers vs. binding declarations

Not every C3 function in a binding module is an `@extern` declaration. Small engine-side convenience macros (`register_component`, `ecs_get`, `ecs_set` in Flecs, or the sizing helpers in Clay) that wrap lower-level extern calls belong in the same module because they're the idiomatic way to use the library — but they're pure C3, no `@extern`, no C symbol. Keep them close to the extern declarations they wrap so the full picture of "how to use this library from C3" lives in one place.

## What idiomatic C3 bindings end up looking like

A call site using one of our bindings should be indistinguishable from a call site using a native C3 library. `sdl::create_window(title, 1920, 1080, sdl::WINDOW_OPENGL)` reads exactly like `std::io::printfn("hello")` — module prefix, snake_case function, PascalCase-or-SCREAMING type references. The fact that one dispatches to a C3 standard-library function and the other marshals down to a C ABI call is an implementation detail the caller never has to think about. That's the goal.