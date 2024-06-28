/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "CrossCarrierSchedulingSCell-SpCell-r17.h"

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
static int
memb_scs30kHz_30kHz_r17_constraint_2(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const BIT_STRING_t *st = (const BIT_STRING_t *)sptr;
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	if(st->size > 0) {
		/* Size in bits */
		size = 8 * st->size - (st->bits_unused & 0x07);
	} else {
		size = 0;
	}
	
	if((size >= 1UL && size <= 496UL)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_scs30kHz_60kHz_r17_constraint_2(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const BIT_STRING_t *st = (const BIT_STRING_t *)sptr;
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	if(st->size > 0) {
		/* Size in bits */
		size = 8 * st->size - (st->bits_unused & 0x07);
	} else {
		size = 0;
	}
	
	if((size >= 1UL && size <= 496UL)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_scs60kHz_60kHz_r17_constraint_2(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const BIT_STRING_t *st = (const BIT_STRING_t *)sptr;
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	if(st->size > 0) {
		/* Size in bits */
		size = 8 * st->size - (st->bits_unused & 0x07);
	} else {
		size = 0;
	}
	
	if((size >= 1UL && size <= 496UL)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_scs15kHz_15kHz_r17_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_scs15kHz_30kHz_r17_constr_5 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_scs15kHz_60kHz_r17_constr_7 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_scs30kHz_30kHz_r17_constr_9 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 9,  9,  1,  496 }	/* (SIZE(1..496)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_scs30kHz_60kHz_r17_constr_10 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 9,  9,  1,  496 }	/* (SIZE(1..496)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_scs60kHz_60kHz_r17_constr_11 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 9,  9,  1,  496 }	/* (SIZE(1..496)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_pdcch_MonitoringOccasion_r17_constr_12 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 1,  1,  0,  1 }	/* (0..1) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_scs15kHz_15kHz_r17_value2enum_3[] = {
	{ 0,	9,	"supported" }
};
static const unsigned int asn_MAP_scs15kHz_15kHz_r17_enum2value_3[] = {
	0	/* supported(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_scs15kHz_15kHz_r17_specs_3 = {
	asn_MAP_scs15kHz_15kHz_r17_value2enum_3,	/* "tag" => N; sorted by tag */
	asn_MAP_scs15kHz_15kHz_r17_enum2value_3,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_scs15kHz_15kHz_r17_tags_3[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_scs15kHz_15kHz_r17_3 = {
	"scs15kHz-15kHz-r17",
	"scs15kHz-15kHz-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_scs15kHz_15kHz_r17_tags_3,
	sizeof(asn_DEF_scs15kHz_15kHz_r17_tags_3)
		/sizeof(asn_DEF_scs15kHz_15kHz_r17_tags_3[0]) - 1, /* 1 */
	asn_DEF_scs15kHz_15kHz_r17_tags_3,	/* Same as above */
	sizeof(asn_DEF_scs15kHz_15kHz_r17_tags_3)
		/sizeof(asn_DEF_scs15kHz_15kHz_r17_tags_3[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_scs15kHz_15kHz_r17_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_scs15kHz_15kHz_r17_specs_3	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_scs15kHz_30kHz_r17_value2enum_5[] = {
	{ 0,	9,	"supported" }
};
static const unsigned int asn_MAP_scs15kHz_30kHz_r17_enum2value_5[] = {
	0	/* supported(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_scs15kHz_30kHz_r17_specs_5 = {
	asn_MAP_scs15kHz_30kHz_r17_value2enum_5,	/* "tag" => N; sorted by tag */
	asn_MAP_scs15kHz_30kHz_r17_enum2value_5,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_scs15kHz_30kHz_r17_tags_5[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_scs15kHz_30kHz_r17_5 = {
	"scs15kHz-30kHz-r17",
	"scs15kHz-30kHz-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_scs15kHz_30kHz_r17_tags_5,
	sizeof(asn_DEF_scs15kHz_30kHz_r17_tags_5)
		/sizeof(asn_DEF_scs15kHz_30kHz_r17_tags_5[0]) - 1, /* 1 */
	asn_DEF_scs15kHz_30kHz_r17_tags_5,	/* Same as above */
	sizeof(asn_DEF_scs15kHz_30kHz_r17_tags_5)
		/sizeof(asn_DEF_scs15kHz_30kHz_r17_tags_5[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_scs15kHz_30kHz_r17_constr_5,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_scs15kHz_30kHz_r17_specs_5	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_scs15kHz_60kHz_r17_value2enum_7[] = {
	{ 0,	9,	"supported" }
};
static const unsigned int asn_MAP_scs15kHz_60kHz_r17_enum2value_7[] = {
	0	/* supported(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_scs15kHz_60kHz_r17_specs_7 = {
	asn_MAP_scs15kHz_60kHz_r17_value2enum_7,	/* "tag" => N; sorted by tag */
	asn_MAP_scs15kHz_60kHz_r17_enum2value_7,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_scs15kHz_60kHz_r17_tags_7[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_scs15kHz_60kHz_r17_7 = {
	"scs15kHz-60kHz-r17",
	"scs15kHz-60kHz-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_scs15kHz_60kHz_r17_tags_7,
	sizeof(asn_DEF_scs15kHz_60kHz_r17_tags_7)
		/sizeof(asn_DEF_scs15kHz_60kHz_r17_tags_7[0]) - 1, /* 1 */
	asn_DEF_scs15kHz_60kHz_r17_tags_7,	/* Same as above */
	sizeof(asn_DEF_scs15kHz_60kHz_r17_tags_7)
		/sizeof(asn_DEF_scs15kHz_60kHz_r17_tags_7[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_scs15kHz_60kHz_r17_constr_7,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_scs15kHz_60kHz_r17_specs_7	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_supportedSCS_Combinations_r17_2[] = {
	{ ATF_POINTER, 6, offsetof(struct CrossCarrierSchedulingSCell_SpCell_r17__supportedSCS_Combinations_r17, scs15kHz_15kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_scs15kHz_15kHz_r17_3,
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
		"scs15kHz-15kHz-r17"
		},
	{ ATF_POINTER, 5, offsetof(struct CrossCarrierSchedulingSCell_SpCell_r17__supportedSCS_Combinations_r17, scs15kHz_30kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_scs15kHz_30kHz_r17_5,
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
		"scs15kHz-30kHz-r17"
		},
	{ ATF_POINTER, 4, offsetof(struct CrossCarrierSchedulingSCell_SpCell_r17__supportedSCS_Combinations_r17, scs15kHz_60kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_scs15kHz_60kHz_r17_7,
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
		"scs15kHz-60kHz-r17"
		},
	{ ATF_POINTER, 3, offsetof(struct CrossCarrierSchedulingSCell_SpCell_r17__supportedSCS_Combinations_r17, scs30kHz_30kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BIT_STRING,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_scs30kHz_30kHz_r17_constr_9,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_scs30kHz_30kHz_r17_constraint_2
		},
		0, 0, /* No default value */
		"scs30kHz-30kHz-r17"
		},
	{ ATF_POINTER, 2, offsetof(struct CrossCarrierSchedulingSCell_SpCell_r17__supportedSCS_Combinations_r17, scs30kHz_60kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BIT_STRING,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_scs30kHz_60kHz_r17_constr_10,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_scs30kHz_60kHz_r17_constraint_2
		},
		0, 0, /* No default value */
		"scs30kHz-60kHz-r17"
		},
	{ ATF_POINTER, 1, offsetof(struct CrossCarrierSchedulingSCell_SpCell_r17__supportedSCS_Combinations_r17, scs60kHz_60kHz_r17),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BIT_STRING,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_scs60kHz_60kHz_r17_constr_11,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_scs60kHz_60kHz_r17_constraint_2
		},
		0, 0, /* No default value */
		"scs60kHz-60kHz-r17"
		},
};
static const int asn_MAP_supportedSCS_Combinations_r17_oms_2[] = { 0, 1, 2, 3, 4, 5 };
static const ber_tlv_tag_t asn_DEF_supportedSCS_Combinations_r17_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_supportedSCS_Combinations_r17_tag2el_2[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* scs15kHz-15kHz-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* scs15kHz-30kHz-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* scs15kHz-60kHz-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* scs30kHz-30kHz-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* scs30kHz-60kHz-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 } /* scs60kHz-60kHz-r17 */
};
static asn_SEQUENCE_specifics_t asn_SPC_supportedSCS_Combinations_r17_specs_2 = {
	sizeof(struct CrossCarrierSchedulingSCell_SpCell_r17__supportedSCS_Combinations_r17),
	offsetof(struct CrossCarrierSchedulingSCell_SpCell_r17__supportedSCS_Combinations_r17, _asn_ctx),
	asn_MAP_supportedSCS_Combinations_r17_tag2el_2,
	6,	/* Count of tags in the map */
	asn_MAP_supportedSCS_Combinations_r17_oms_2,	/* Optional members */
	6, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_supportedSCS_Combinations_r17_2 = {
	"supportedSCS-Combinations-r17",
	"supportedSCS-Combinations-r17",
	&asn_OP_SEQUENCE,
	asn_DEF_supportedSCS_Combinations_r17_tags_2,
	sizeof(asn_DEF_supportedSCS_Combinations_r17_tags_2)
		/sizeof(asn_DEF_supportedSCS_Combinations_r17_tags_2[0]) - 1, /* 1 */
	asn_DEF_supportedSCS_Combinations_r17_tags_2,	/* Same as above */
	sizeof(asn_DEF_supportedSCS_Combinations_r17_tags_2)
		/sizeof(asn_DEF_supportedSCS_Combinations_r17_tags_2[0]), /* 2 */
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
	asn_MBR_supportedSCS_Combinations_r17_2,
	6,	/* Elements count */
	&asn_SPC_supportedSCS_Combinations_r17_specs_2	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_pdcch_MonitoringOccasion_r17_value2enum_12[] = {
	{ 0,	4,	"val1" },
	{ 1,	4,	"val2" }
};
static const unsigned int asn_MAP_pdcch_MonitoringOccasion_r17_enum2value_12[] = {
	0,	/* val1(0) */
	1	/* val2(1) */
};
static const asn_INTEGER_specifics_t asn_SPC_pdcch_MonitoringOccasion_r17_specs_12 = {
	asn_MAP_pdcch_MonitoringOccasion_r17_value2enum_12,	/* "tag" => N; sorted by tag */
	asn_MAP_pdcch_MonitoringOccasion_r17_enum2value_12,	/* N => "tag"; sorted by N */
	2,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_pdcch_MonitoringOccasion_r17_tags_12[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_pdcch_MonitoringOccasion_r17_12 = {
	"pdcch-MonitoringOccasion-r17",
	"pdcch-MonitoringOccasion-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_pdcch_MonitoringOccasion_r17_tags_12,
	sizeof(asn_DEF_pdcch_MonitoringOccasion_r17_tags_12)
		/sizeof(asn_DEF_pdcch_MonitoringOccasion_r17_tags_12[0]) - 1, /* 1 */
	asn_DEF_pdcch_MonitoringOccasion_r17_tags_12,	/* Same as above */
	sizeof(asn_DEF_pdcch_MonitoringOccasion_r17_tags_12)
		/sizeof(asn_DEF_pdcch_MonitoringOccasion_r17_tags_12[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_pdcch_MonitoringOccasion_r17_constr_12,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_pdcch_MonitoringOccasion_r17_specs_12	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_CrossCarrierSchedulingSCell_SpCell_r17_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct CrossCarrierSchedulingSCell_SpCell_r17, supportedSCS_Combinations_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		0,
		&asn_DEF_supportedSCS_Combinations_r17_2,
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
		"supportedSCS-Combinations-r17"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CrossCarrierSchedulingSCell_SpCell_r17, pdcch_MonitoringOccasion_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_pdcch_MonitoringOccasion_r17_12,
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
		"pdcch-MonitoringOccasion-r17"
		},
};
static const ber_tlv_tag_t asn_DEF_CrossCarrierSchedulingSCell_SpCell_r17_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_CrossCarrierSchedulingSCell_SpCell_r17_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* supportedSCS-Combinations-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* pdcch-MonitoringOccasion-r17 */
};
asn_SEQUENCE_specifics_t asn_SPC_CrossCarrierSchedulingSCell_SpCell_r17_specs_1 = {
	sizeof(struct CrossCarrierSchedulingSCell_SpCell_r17),
	offsetof(struct CrossCarrierSchedulingSCell_SpCell_r17, _asn_ctx),
	asn_MAP_CrossCarrierSchedulingSCell_SpCell_r17_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_CrossCarrierSchedulingSCell_SpCell_r17 = {
	"CrossCarrierSchedulingSCell-SpCell-r17",
	"CrossCarrierSchedulingSCell-SpCell-r17",
	&asn_OP_SEQUENCE,
	asn_DEF_CrossCarrierSchedulingSCell_SpCell_r17_tags_1,
	sizeof(asn_DEF_CrossCarrierSchedulingSCell_SpCell_r17_tags_1)
		/sizeof(asn_DEF_CrossCarrierSchedulingSCell_SpCell_r17_tags_1[0]), /* 1 */
	asn_DEF_CrossCarrierSchedulingSCell_SpCell_r17_tags_1,	/* Same as above */
	sizeof(asn_DEF_CrossCarrierSchedulingSCell_SpCell_r17_tags_1)
		/sizeof(asn_DEF_CrossCarrierSchedulingSCell_SpCell_r17_tags_1[0]), /* 1 */
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
	asn_MBR_CrossCarrierSchedulingSCell_SpCell_r17_1,
	2,	/* Elements count */
	&asn_SPC_CrossCarrierSchedulingSCell_SpCell_r17_specs_1	/* Additional specs */
};

