/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "SIB1-v1630-IEs.h"

static int
memb_uac_AC1_SelectAssistInfo_r16_constraint_2(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Determine the number of elements */
	size = _A_CSEQUENCE_FROM_VOID(sptr)->count;
	
	if((size >= 2UL && size <= 12UL)) {
		/* Perform validation of the inner elements */
		return SEQUENCE_OF_constraint(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_uac_AC1_SelectAssistInfo_r16_constr_3 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 4,  4,  2,  12 }	/* (SIZE(2..12)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_uac_AC1_SelectAssistInfo_r16_constr_3 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 4,  4,  2,  12 }	/* (SIZE(2..12)) */,
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static asn_TYPE_member_t asn_MBR_uac_AC1_SelectAssistInfo_r16_3[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (10 << 2)),
		0,
		&asn_DEF_UAC_AC1_SelectAssistInfo_r16,
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
		""
		},
};
static const ber_tlv_tag_t asn_DEF_uac_AC1_SelectAssistInfo_r16_tags_3[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_uac_AC1_SelectAssistInfo_r16_specs_3 = {
	sizeof(struct SIB1_v1630_IEs__uac_BarringInfo_v1630__uac_AC1_SelectAssistInfo_r16),
	offsetof(struct SIB1_v1630_IEs__uac_BarringInfo_v1630__uac_AC1_SelectAssistInfo_r16, _asn_ctx),
	1,	/* XER encoding is XMLValueList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_uac_AC1_SelectAssistInfo_r16_3 = {
	"uac-AC1-SelectAssistInfo-r16",
	"uac-AC1-SelectAssistInfo-r16",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_uac_AC1_SelectAssistInfo_r16_tags_3,
	sizeof(asn_DEF_uac_AC1_SelectAssistInfo_r16_tags_3)
		/sizeof(asn_DEF_uac_AC1_SelectAssistInfo_r16_tags_3[0]) - 1, /* 1 */
	asn_DEF_uac_AC1_SelectAssistInfo_r16_tags_3,	/* Same as above */
	sizeof(asn_DEF_uac_AC1_SelectAssistInfo_r16_tags_3)
		/sizeof(asn_DEF_uac_AC1_SelectAssistInfo_r16_tags_3[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_uac_AC1_SelectAssistInfo_r16_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_OF_constraint
	},
	asn_MBR_uac_AC1_SelectAssistInfo_r16_3,
	1,	/* Single element */
	&asn_SPC_uac_AC1_SelectAssistInfo_r16_specs_3	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_uac_BarringInfo_v1630_2[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct SIB1_v1630_IEs__uac_BarringInfo_v1630, uac_AC1_SelectAssistInfo_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		0,
		&asn_DEF_uac_AC1_SelectAssistInfo_r16_3,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_uac_AC1_SelectAssistInfo_r16_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_uac_AC1_SelectAssistInfo_r16_constraint_2
		},
		0, 0, /* No default value */
		"uac-AC1-SelectAssistInfo-r16"
		},
};
static const ber_tlv_tag_t asn_DEF_uac_BarringInfo_v1630_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_uac_BarringInfo_v1630_tag2el_2[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 } /* uac-AC1-SelectAssistInfo-r16 */
};
static asn_SEQUENCE_specifics_t asn_SPC_uac_BarringInfo_v1630_specs_2 = {
	sizeof(struct SIB1_v1630_IEs__uac_BarringInfo_v1630),
	offsetof(struct SIB1_v1630_IEs__uac_BarringInfo_v1630, _asn_ctx),
	asn_MAP_uac_BarringInfo_v1630_tag2el_2,
	1,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_uac_BarringInfo_v1630_2 = {
	"uac-BarringInfo-v1630",
	"uac-BarringInfo-v1630",
	&asn_OP_SEQUENCE,
	asn_DEF_uac_BarringInfo_v1630_tags_2,
	sizeof(asn_DEF_uac_BarringInfo_v1630_tags_2)
		/sizeof(asn_DEF_uac_BarringInfo_v1630_tags_2[0]) - 1, /* 1 */
	asn_DEF_uac_BarringInfo_v1630_tags_2,	/* Same as above */
	sizeof(asn_DEF_uac_BarringInfo_v1630_tags_2)
		/sizeof(asn_DEF_uac_BarringInfo_v1630_tags_2[0]), /* 2 */
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
	asn_MBR_uac_BarringInfo_v1630_2,
	1,	/* Elements count */
	&asn_SPC_uac_BarringInfo_v1630_specs_2	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_SIB1_v1630_IEs_1[] = {
	{ ATF_POINTER, 2, offsetof(struct SIB1_v1630_IEs, uac_BarringInfo_v1630),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		0,
		&asn_DEF_uac_BarringInfo_v1630_2,
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
		"uac-BarringInfo-v1630"
		},
	{ ATF_POINTER, 1, offsetof(struct SIB1_v1630_IEs, nonCriticalExtension),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SIB1_v1700_IEs,
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
		"nonCriticalExtension"
		},
};
static const int asn_MAP_SIB1_v1630_IEs_oms_1[] = { 0, 1 };
static const ber_tlv_tag_t asn_DEF_SIB1_v1630_IEs_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_SIB1_v1630_IEs_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* uac-BarringInfo-v1630 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* nonCriticalExtension */
};
asn_SEQUENCE_specifics_t asn_SPC_SIB1_v1630_IEs_specs_1 = {
	sizeof(struct SIB1_v1630_IEs),
	offsetof(struct SIB1_v1630_IEs, _asn_ctx),
	asn_MAP_SIB1_v1630_IEs_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_SIB1_v1630_IEs_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_SIB1_v1630_IEs = {
	"SIB1-v1630-IEs",
	"SIB1-v1630-IEs",
	&asn_OP_SEQUENCE,
	asn_DEF_SIB1_v1630_IEs_tags_1,
	sizeof(asn_DEF_SIB1_v1630_IEs_tags_1)
		/sizeof(asn_DEF_SIB1_v1630_IEs_tags_1[0]), /* 1 */
	asn_DEF_SIB1_v1630_IEs_tags_1,	/* Same as above */
	sizeof(asn_DEF_SIB1_v1630_IEs_tags_1)
		/sizeof(asn_DEF_SIB1_v1630_IEs_tags_1[0]), /* 1 */
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
	asn_MBR_SIB1_v1630_IEs_1,
	2,	/* Elements count */
	&asn_SPC_SIB1_v1630_IEs_specs_1	/* Additional specs */
};

