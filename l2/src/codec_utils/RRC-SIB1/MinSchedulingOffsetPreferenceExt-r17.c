/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "MinSchedulingOffsetPreferenceExt-r17.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_preferredK0_SCS_480kHz_r17_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  3 }	/* (0..3) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_preferredK0_SCS_960kHz_r17_constr_8 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  3 }	/* (0..3) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_preferredK2_SCS_480kHz_r17_constr_14 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  3 }	/* (0..3) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_preferredK2_SCS_960kHz_r17_constr_19 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  3 }	/* (0..3) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_preferredK0_SCS_480kHz_r17_value2enum_3[] = {
	{ 0,	3,	"sl8" },
	{ 1,	4,	"sl16" },
	{ 2,	4,	"sl32" },
	{ 3,	4,	"sl48" }
};
static const unsigned int asn_MAP_preferredK0_SCS_480kHz_r17_enum2value_3[] = {
	1,	/* sl16(1) */
	2,	/* sl32(2) */
	3,	/* sl48(3) */
	0	/* sl8(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_preferredK0_SCS_480kHz_r17_specs_3 = {
	asn_MAP_preferredK0_SCS_480kHz_r17_value2enum_3,	/* "tag" => N; sorted by tag */
	asn_MAP_preferredK0_SCS_480kHz_r17_enum2value_3,	/* N => "tag"; sorted by N */
	4,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_preferredK0_SCS_480kHz_r17_tags_3[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_preferredK0_SCS_480kHz_r17_3 = {
	"preferredK0-SCS-480kHz-r17",
	"preferredK0-SCS-480kHz-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_preferredK0_SCS_480kHz_r17_tags_3,
	sizeof(asn_DEF_preferredK0_SCS_480kHz_r17_tags_3)
		/sizeof(asn_DEF_preferredK0_SCS_480kHz_r17_tags_3[0]) - 1, /* 1 */
	asn_DEF_preferredK0_SCS_480kHz_r17_tags_3,	/* Same as above */
	sizeof(asn_DEF_preferredK0_SCS_480kHz_r17_tags_3)
		/sizeof(asn_DEF_preferredK0_SCS_480kHz_r17_tags_3[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_preferredK0_SCS_480kHz_r17_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_preferredK0_SCS_480kHz_r17_specs_3	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_preferredK0_SCS_960kHz_r17_value2enum_8[] = {
	{ 0,	3,	"sl8" },
	{ 1,	4,	"sl16" },
	{ 2,	4,	"sl32" },
	{ 3,	4,	"sl48" }
};
static const unsigned int asn_MAP_preferredK0_SCS_960kHz_r17_enum2value_8[] = {
	1,	/* sl16(1) */
	2,	/* sl32(2) */
	3,	/* sl48(3) */
	0	/* sl8(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_preferredK0_SCS_960kHz_r17_specs_8 = {
	asn_MAP_preferredK0_SCS_960kHz_r17_value2enum_8,	/* "tag" => N; sorted by tag */
	asn_MAP_preferredK0_SCS_960kHz_r17_enum2value_8,	/* N => "tag"; sorted by N */
	4,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_preferredK0_SCS_960kHz_r17_tags_8[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_preferredK0_SCS_960kHz_r17_8 = {
	"preferredK0-SCS-960kHz-r17",
	"preferredK0-SCS-960kHz-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_preferredK0_SCS_960kHz_r17_tags_8,
	sizeof(asn_DEF_preferredK0_SCS_960kHz_r17_tags_8)
		/sizeof(asn_DEF_preferredK0_SCS_960kHz_r17_tags_8[0]) - 1, /* 1 */
	asn_DEF_preferredK0_SCS_960kHz_r17_tags_8,	/* Same as above */
	sizeof(asn_DEF_preferredK0_SCS_960kHz_r17_tags_8)
		/sizeof(asn_DEF_preferredK0_SCS_960kHz_r17_tags_8[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_preferredK0_SCS_960kHz_r17_constr_8,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_preferredK0_SCS_960kHz_r17_specs_8	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_preferredK0_r17_2[] = {
	{ ATF_POINTER, 2, offsetof(struct MinSchedulingOffsetPreferenceExt_r17__preferredK0_r17, preferredK0_SCS_480kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_preferredK0_SCS_480kHz_r17_3,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"preferredK0-SCS-480kHz-r17"
		},
	{ ATF_POINTER, 1, offsetof(struct MinSchedulingOffsetPreferenceExt_r17__preferredK0_r17, preferredK0_SCS_960kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_preferredK0_SCS_960kHz_r17_8,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"preferredK0-SCS-960kHz-r17"
		},
};
static const int asn_MAP_preferredK0_r17_oms_2[] = { 0, 1 };
static const ber_tlv_tag_t asn_DEF_preferredK0_r17_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_preferredK0_r17_tag2el_2[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* preferredK0-SCS-480kHz-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* preferredK0-SCS-960kHz-r17 */
};
static asn_SEQUENCE_specifics_t asn_SPC_preferredK0_r17_specs_2 = {
	sizeof(struct MinSchedulingOffsetPreferenceExt_r17__preferredK0_r17),
	offsetof(struct MinSchedulingOffsetPreferenceExt_r17__preferredK0_r17, _asn_ctx),
	asn_MAP_preferredK0_r17_tag2el_2,
	2,	/* Count of tags in the map */
	asn_MAP_preferredK0_r17_oms_2,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_preferredK0_r17_2 = {
	"preferredK0-r17",
	"preferredK0-r17",
	&asn_OP_SEQUENCE,
	asn_DEF_preferredK0_r17_tags_2,
	sizeof(asn_DEF_preferredK0_r17_tags_2)
		/sizeof(asn_DEF_preferredK0_r17_tags_2[0]) - 1, /* 1 */
	asn_DEF_preferredK0_r17_tags_2,	/* Same as above */
	sizeof(asn_DEF_preferredK0_r17_tags_2)
		/sizeof(asn_DEF_preferredK0_r17_tags_2[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_preferredK0_r17_2,
	2,	/* Elements count */
	&asn_SPC_preferredK0_r17_specs_2	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_preferredK2_SCS_480kHz_r17_value2enum_14[] = {
	{ 0,	3,	"sl8" },
	{ 1,	4,	"sl16" },
	{ 2,	4,	"sl32" },
	{ 3,	4,	"sl48" }
};
static const unsigned int asn_MAP_preferredK2_SCS_480kHz_r17_enum2value_14[] = {
	1,	/* sl16(1) */
	2,	/* sl32(2) */
	3,	/* sl48(3) */
	0	/* sl8(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_preferredK2_SCS_480kHz_r17_specs_14 = {
	asn_MAP_preferredK2_SCS_480kHz_r17_value2enum_14,	/* "tag" => N; sorted by tag */
	asn_MAP_preferredK2_SCS_480kHz_r17_enum2value_14,	/* N => "tag"; sorted by N */
	4,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_preferredK2_SCS_480kHz_r17_tags_14[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_preferredK2_SCS_480kHz_r17_14 = {
	"preferredK2-SCS-480kHz-r17",
	"preferredK2-SCS-480kHz-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_preferredK2_SCS_480kHz_r17_tags_14,
	sizeof(asn_DEF_preferredK2_SCS_480kHz_r17_tags_14)
		/sizeof(asn_DEF_preferredK2_SCS_480kHz_r17_tags_14[0]) - 1, /* 1 */
	asn_DEF_preferredK2_SCS_480kHz_r17_tags_14,	/* Same as above */
	sizeof(asn_DEF_preferredK2_SCS_480kHz_r17_tags_14)
		/sizeof(asn_DEF_preferredK2_SCS_480kHz_r17_tags_14[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_preferredK2_SCS_480kHz_r17_constr_14,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_preferredK2_SCS_480kHz_r17_specs_14	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_preferredK2_SCS_960kHz_r17_value2enum_19[] = {
	{ 0,	3,	"sl8" },
	{ 1,	4,	"sl16" },
	{ 2,	4,	"sl32" },
	{ 3,	4,	"sl48" }
};
static const unsigned int asn_MAP_preferredK2_SCS_960kHz_r17_enum2value_19[] = {
	1,	/* sl16(1) */
	2,	/* sl32(2) */
	3,	/* sl48(3) */
	0	/* sl8(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_preferredK2_SCS_960kHz_r17_specs_19 = {
	asn_MAP_preferredK2_SCS_960kHz_r17_value2enum_19,	/* "tag" => N; sorted by tag */
	asn_MAP_preferredK2_SCS_960kHz_r17_enum2value_19,	/* N => "tag"; sorted by N */
	4,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_preferredK2_SCS_960kHz_r17_tags_19[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_preferredK2_SCS_960kHz_r17_19 = {
	"preferredK2-SCS-960kHz-r17",
	"preferredK2-SCS-960kHz-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_preferredK2_SCS_960kHz_r17_tags_19,
	sizeof(asn_DEF_preferredK2_SCS_960kHz_r17_tags_19)
		/sizeof(asn_DEF_preferredK2_SCS_960kHz_r17_tags_19[0]) - 1, /* 1 */
	asn_DEF_preferredK2_SCS_960kHz_r17_tags_19,	/* Same as above */
	sizeof(asn_DEF_preferredK2_SCS_960kHz_r17_tags_19)
		/sizeof(asn_DEF_preferredK2_SCS_960kHz_r17_tags_19[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_preferredK2_SCS_960kHz_r17_constr_19,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_preferredK2_SCS_960kHz_r17_specs_19	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_preferredK2_r17_13[] = {
	{ ATF_POINTER, 2, offsetof(struct MinSchedulingOffsetPreferenceExt_r17__preferredK2_r17, preferredK2_SCS_480kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_preferredK2_SCS_480kHz_r17_14,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"preferredK2-SCS-480kHz-r17"
		},
	{ ATF_POINTER, 1, offsetof(struct MinSchedulingOffsetPreferenceExt_r17__preferredK2_r17, preferredK2_SCS_960kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_preferredK2_SCS_960kHz_r17_19,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"preferredK2-SCS-960kHz-r17"
		},
};
static const int asn_MAP_preferredK2_r17_oms_13[] = { 0, 1 };
static const ber_tlv_tag_t asn_DEF_preferredK2_r17_tags_13[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_preferredK2_r17_tag2el_13[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* preferredK2-SCS-480kHz-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* preferredK2-SCS-960kHz-r17 */
};
static asn_SEQUENCE_specifics_t asn_SPC_preferredK2_r17_specs_13 = {
	sizeof(struct MinSchedulingOffsetPreferenceExt_r17__preferredK2_r17),
	offsetof(struct MinSchedulingOffsetPreferenceExt_r17__preferredK2_r17, _asn_ctx),
	asn_MAP_preferredK2_r17_tag2el_13,
	2,	/* Count of tags in the map */
	asn_MAP_preferredK2_r17_oms_13,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_preferredK2_r17_13 = {
	"preferredK2-r17",
	"preferredK2-r17",
	&asn_OP_SEQUENCE,
	asn_DEF_preferredK2_r17_tags_13,
	sizeof(asn_DEF_preferredK2_r17_tags_13)
		/sizeof(asn_DEF_preferredK2_r17_tags_13[0]) - 1, /* 1 */
	asn_DEF_preferredK2_r17_tags_13,	/* Same as above */
	sizeof(asn_DEF_preferredK2_r17_tags_13)
		/sizeof(asn_DEF_preferredK2_r17_tags_13[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_preferredK2_r17_13,
	2,	/* Elements count */
	&asn_SPC_preferredK2_r17_specs_13	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_MinSchedulingOffsetPreferenceExt_r17_1[] = {
	{ ATF_POINTER, 2, offsetof(struct MinSchedulingOffsetPreferenceExt_r17, preferredK0_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		0,
		&asn_DEF_preferredK0_r17_2,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"preferredK0-r17"
		},
	{ ATF_POINTER, 1, offsetof(struct MinSchedulingOffsetPreferenceExt_r17, preferredK2_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		0,
		&asn_DEF_preferredK2_r17_13,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"preferredK2-r17"
		},
};
static const int asn_MAP_MinSchedulingOffsetPreferenceExt_r17_oms_1[] = { 0, 1 };
static const ber_tlv_tag_t asn_DEF_MinSchedulingOffsetPreferenceExt_r17_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_MinSchedulingOffsetPreferenceExt_r17_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* preferredK0-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* preferredK2-r17 */
};
asn_SEQUENCE_specifics_t asn_SPC_MinSchedulingOffsetPreferenceExt_r17_specs_1 = {
	sizeof(struct MinSchedulingOffsetPreferenceExt_r17),
	offsetof(struct MinSchedulingOffsetPreferenceExt_r17, _asn_ctx),
	asn_MAP_MinSchedulingOffsetPreferenceExt_r17_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_MinSchedulingOffsetPreferenceExt_r17_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_MinSchedulingOffsetPreferenceExt_r17 = {
	"MinSchedulingOffsetPreferenceExt-r17",
	"MinSchedulingOffsetPreferenceExt-r17",
	&asn_OP_SEQUENCE,
	asn_DEF_MinSchedulingOffsetPreferenceExt_r17_tags_1,
	sizeof(asn_DEF_MinSchedulingOffsetPreferenceExt_r17_tags_1)
		/sizeof(asn_DEF_MinSchedulingOffsetPreferenceExt_r17_tags_1[0]), /* 1 */
	asn_DEF_MinSchedulingOffsetPreferenceExt_r17_tags_1,	/* Same as above */
	sizeof(asn_DEF_MinSchedulingOffsetPreferenceExt_r17_tags_1)
		/sizeof(asn_DEF_MinSchedulingOffsetPreferenceExt_r17_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_MinSchedulingOffsetPreferenceExt_r17_1,
	2,	/* Elements count */
	&asn_SPC_MinSchedulingOffsetPreferenceExt_r17_specs_1	/* Additional specs */
};

