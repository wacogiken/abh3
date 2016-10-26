#include "Python.h"
#include "pack_float.h"

/*  PACK_FLOAT型を作成する関数  */
PyObject* pkflt_make_pkflt(PyObject* self, PyObject* args)
{
    PACK_FLOAT_DUAL pkflt;
    long _kasu, _ten, _sisu;

    if (!PyArg_ParseTuple(args, "kkk", &_kasu, &_ten, &_sisu)) {
        return NULL;
    }
    pkflt.pack = make_pkflt(_kasu, _ten, _sisu);
    return Py_BuildValue("k", pkflt.value);
}

/*  数値文字列 → PACK_FLOAT型 変換関数 */
PyObject* pkflt_str_to_pkflt(PyObject* self, PyObject* args)
{
    PACK_FLOAT_DUAL pkflt;
    char *s;

    if (!PyArg_ParseTuple(args, "s", &s)) {
        return NULL;
    }
    if (str_to_pkflt(s, &pkflt.pack)) {
        return NULL;
    }
    else {
        return Py_BuildValue("k", pkflt.value);
    }
}

/*  PACK_FLOAT型 → 数値文字列 変換関数 */
PyObject* pkflt_pkflt_to_str(PyObject* self, PyObject* args)
{
    PACK_FLOAT_DUAL pkflt;
    char s[64];

    if (!PyArg_ParseTuple(args, "k", &pkflt.value)) {
        return NULL;
    }
    pkflt_to_str(pkflt.pack, s);
    return Py_BuildValue("s", s);
}

/*  PACK_FLOAT型 → float型 変換関数    */
PyObject* pkflt_pkflt_to_flt(PyObject* self, PyObject* args)
{
    PACK_FLOAT_DUAL pkflt;
    float f;

    if (!PyArg_ParseTuple(args, "k", &pkflt.value)) {
        return NULL;
    }
    f = pkflt_to_flt(pkflt.pack);
    return Py_BuildValue("f", f);
}

/*  float型 → PACK_FLOAT型 変換関数    */
PyObject* pkflt_flt_to_pkflt(PyObject* self, PyObject* args)
{
    PACK_FLOAT_DUAL pkflt;
    float f;

    if (!PyArg_ParseTuple(args, "f", &f)) {
        return NULL;
    }
    pkflt.pack = flt_to_pkflt(f);
    return Py_BuildValue("k", pkflt.value);
}

static PyMethodDef pkfltmethods[] = {
    {"make_pkflt",   pkflt_make_pkflt,   METH_VARARGS},
    {"str_to_pkflt", pkflt_str_to_pkflt, METH_VARARGS},
    {"pkflt_to_str", pkflt_pkflt_to_str, METH_VARARGS},
    {"pkflt_to_flt", pkflt_pkflt_to_flt, METH_VARARGS},
    {"flt_to_pkflt", pkflt_flt_to_pkflt, METH_VARARGS},
    {NULL},
};

void initpkflt()
{
    Py_InitModule("pkflt", pkfltmethods);
}

