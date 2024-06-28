/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "ULTxSwitchingBandPair-r16.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_bandIndexUL1_r16_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 1L && value <= 32L)) {
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
memb_bandIndexUL2_r16_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 1L && value <= 32L)) {
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
memb_uplinkTxSwitching_DL_Interruption_r16_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
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
	
	if((size >= 1UL && size <= 32UL)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_uplinkTxSwitchingPeriod_r16_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  2 }	/* (0..2) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_bandIndexUL1_r16_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 5,  5,  1,  32 }	/* (1..32) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_bandIndexUL2_r16_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 5,  5,  1,  32 }	/* (1..32) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_uplinkTxSwitching_DL_Interruption_r16_constr_8 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 5,  5,  1,  32 }	/* (SIZE(1..32)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_uplinkTxSwitchingPeriod_r16_value2enum_4[] = {
	{ 0,	5,	"n35us" },
	{ 1,	6,	"n140us" },
	{ 2,	6,	"n210us" }
};
static const unsigned int asn_MAP_uplinkTxSwitchingPeriod_r16_enum2value_4[] = {
	1,	/* n140us(1) */
	2,	/* n210us(2) */
	0	/* n35us(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_uplinkTxSwitchingPeriod_r16_specs_4 = {
	asn_MAP_uplinkTxSwitchingPeriod_r16_value2enum_4,	/* "tag" => N; sorted by tag */
	asn_MAP_uplinkTxSwitchingPeriod_r16_enum2value_4,	/* N => "tag"; sorted by N */
	3,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_uplinkTxSwitchingPeriod_r16_tags_4[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_uplinkTxSwitchingPeriod_r16_4 = {
	"uplinkTxSwitchingPeriod-r16",
	"uplinkTxSwitchingPeriod-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_uplinkTxSwitchingPeriod_r16_tags_4,
	sizeof(asn_DEF_uplinkTxSwitchingPeriod_r16_tags_4)
		/sizeof(asn_DEF_uplinkTxSwitchingPeriod_r16_tags_4[0]) - 1, /* 1 */
	asn_DEF_uplinkTxSwitchingPeriod_r16_tags_4,	/* Same as above */
	sizeof(asn_DEF_uplinkTxSwitchingPeriod_r16_tags_4)
		/sizeof(asn_DEF_uplinkTxSwitchingPeriod_r16_tags_4[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_uplinkTxSwitchingPeriod_r16_constr_4,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_uplinkTxSwitchingPeriod_r16_specs_4	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_ULTxSwitchingBandPair_r16_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct ULTxSwitchingBandPair_r16, bandIndexUL1_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_bandIndexUL1_r16_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_bandIndexUL1_r16_constraint_1
		},
		0, 0, /* No default value */
		"bandIndexUL1-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ULTxSwitchingBandPair_r16, bandIndexUL2_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_bandIndexUL2_r16_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_bandIndexUL2_r16_constraint_1
		},
		0, 0, /* No default value */
		"bandIndexUL2-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ULTxSwitchingBandPair_r16, uplinkTxSwitchingPeriod_r16),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_uplinkTxSwitchingPeriod_r16_4,
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
		"uplinkTxSwitchingPeriod-r16"
		},
	{ ATF_POINTER, 1, offsetof(struct ULTxSwitchingBandPair_r16, uplinkTxSwitching_DL_Interruption_r16),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BIT_STRING,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_uplinkTxSwitching_DL_Interruption_r16_constr_8,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_uplinkTxSwitching_DL_Interruption_r16_constraint_1
		},
		0, 0, /* No default value */
		"uplinkTxSwitching-DL-Interruption-r16"
		},
};
static const int asn_MAP_ULTxSwitchingBandPair_r16_oms_1[] = { 3 };
static const ber_tlv_tag_t asn_DEF_ULTxSwitchingBandPair_r16_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ULTxSwitchingBandPair_r16_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* bandIndexUL1-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* bandIndexUL2-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* uplinkTxSwitchingPeriod-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* uplinkTxSwitching-DL-Interruption-r16 */
};
asn_SEQUENCE_specifics_t asn_SPC_ULTxSwitchingBandPair_r16_specs_1 = {
	sizeof(struct ULTxSwitchingBandPair_r16),
	offsetof(struct ULTxSwitchingBandPair_r16, _asn_ctx),
	asn_MAP_ULTxSwitchingBandPair_r16_tag2el_1,
	4,	/* Count of tags in the map */
	asn_MAP_ULTxSwitchingBandPair_r16_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_ULTxSwitchingBandPair_r16 = {
	"ULTxSwitchingBandPair-r16",
	"ULTxSwitchingBandPair-r16",
	&asn_OP_SEQUENCE,
	asn_DEF_ULTxSwitchingBandPair_r16_tags_1,
	sizeof(asn_DEF_ULTxSwitchingBandPair_r16_tags_1)
		/sizeof(asn_DEF_ULTxSwitchingBandPair_r16_tags_1[0]), /* 1 */
	asn_DEF_ULTxSwitchingBandPair_r16_tags_1,	/* Same as above */
	sizeof(asn_DEF_ULTxSwitchingBandPair_r16_tags_1)
		/sizeof(asn_DEF_ULTxSwitchingBandPair_r16_tags_1[0]), /* 1 */
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
	asn_MBR_ULTxSwitchingBandPair_r16_1,
	4,	/* Elements count */
	&asn_SPC_ULTxSwitchingBandPair_r16_specs_1	/* Additional specs */
};

