# vtp1 — vendored from the VTP/1 reference implementation

Four files, copied unmodified from
[Lapsmith-app/VTP](https://github.com/Lapsmith-app/VTP) at commit `6a67515`
(2026-09-01, spec v0.9-rc):

| File | From | Why it is here |
|---|---|---|
| `vtp1.h` | `reference/c/vtp1.h` | the record structs the encoder takes |
| `vtp1_generated.h` | `reference/c/vtp1_generated.h` | UUIDs, offsets, sizes and bitmasks, generated from `schema/vtp1.yaml` |
| `vtp1_encode.h` | `reference/c/vtp1_encode.h` | the device-side API |
| `vtp1_encode.c` | `reference/c/vtp1_encode.c` | the encoder itself — C99, no dependencies past `string.h` |

The encoder is here rather than hand-written because it is the code the
conformance corpus tests. A batch this firmware puts on the wire is byte-for-byte
what `conformance/` says a batch is, and the rules that are easy to get subtly
wrong — record 0's `dt` being zero, an eleven-bit identifier that does not fit in
eleven bits, reserved bits written as anything but zero — are refused here rather
than shipped and found in the field.

`vtp1.h` declares the decoder functions too. Nothing in this firmware calls them
and `vtp1.c` is deliberately not vendored: a device encodes, and the only thing
it parses is a Control request, which is `[opcode][tag][params]` and is read by
hand in `src/vtpsvc.cpp`.

## Refreshing

    git -C ../VTP log --oneline -1        # note the commit
    cp ../VTP/reference/c/vtp1.h ../VTP/reference/c/vtp1_generated.h \
       ../VTP/reference/c/vtp1_encode.h ../VTP/reference/c/vtp1_encode.c lib/vtp1/

Then update the commit above. Do not edit these files in place — a local fix
here is a fix the upstream corpus never sees, and the next refresh silently
reverts it.
